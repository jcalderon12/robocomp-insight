"""GraphDB writes off the agent's main thread.

The agent stops the robot, validates every change of the working memory and drives the loop on Qt's
main thread. A GraphDB write there held all of it for as long as GraphDB took to answer: with the
repository's inference on, 5 s timeouts, and the robot stopped 5 s after losing the bottle (09/10).
The agent queues its writes here instead. One thread applies them in the order they were queued and
retries a failed one before going on, so a write never overtakes an earlier one (a case's episode
never lands before the mirror is restored). A read of the semantic memory goes through the same
queue (call), so it sees every write queued before it.
"""
from __future__ import annotations

import queue
import threading
import time
from typing import Callable, Optional

#: Attempts per write, and the wait between them. Every write is idempotent (graph replacements,
#: INSERT/DELETE DATA, and Turtle without blank nodes), so repeating one after a timeout is safe.
ATTEMPTS = 3
RETRY_SECONDS = 5.0


class GraphDBWriter:
    def __init__(self, client, log: Callable[[str, str], None], *, mirror=frozenset(),
                 attempts: int = ATTEMPTS, retry_seconds: float = RETRY_SECONDS):
        self.client = client
        self._log = log
        self._attempts = attempts
        self._retry_seconds = retry_seconds
        # The mirror of the working memory GraphDB is known to hold. A delta is computed against it
        # when it runs, so the next sync makes up for one that failed for good.
        self._mirror = frozenset(mirror)
        self._jobs: queue.Queue = queue.Queue()
        threading.Thread(target=self._run, name="graphdb-writer", daemon=True).start()

    def submit(self, what: str, write: Callable, on_failure: Callable | None = None) -> None:
        """Queue write(client), which may return a message to log once done. `what` names it in the
        log; on_failure() runs if every attempt fails."""
        self._jobs.put((what, write, on_failure))

    def sync_mirror(self, target) -> None:
        """Bring the mirror in the live graph to `target`; the rest of the live graph is left alone."""
        target = frozenset(target)

        def write(client):
            added, removed = target - self._mirror, self._mirror - target
            client.apply_delta(added=added, removed=removed)
            self._mirror = target
            return (f"Semantic mirror synchronized to GraphDB with +{len(added)} / -{len(removed)} "
                    f"changes ({len(target)} current triples).")
        self.submit("synchronize the semantic mirror", write)

    def restore_mirror(self, mirror) -> None:
        """The live graph back to the mirror of the working memory alone. The graphs of the cases stay:
        GraphDB is the long-term memory (decision of 08/10)."""
        mirror = frozenset(mirror)

        def write(client):
            client.replace_with_triples(mirror)
            self._mirror = mirror
            return "Live graph restored to the mirror of the working memory; previous cases kept."
        self.submit("restore the mirror", write)

    def call(self, what: str, operation: Callable, timeout_s: Optional[float] = None):
        """Run operation(client) after every write queued before it, with the same retries, and return
        what it returns. Raises the last error if every attempt fails, or TimeoutError."""
        done, outcome = threading.Event(), {}

        def run(client):
            try:
                outcome["value"] = operation(client)
            except Exception as error:      # kept for the caller; the writer retries and logs it
                outcome["error"] = error
                raise
            outcome.pop("error", None)
            done.set()
        self._jobs.put((what, run, done.set))
        if not done.wait(timeout_s):
            raise TimeoutError(f"GraphDB did not {what} within {timeout_s:g} s")
        if "error" in outcome:
            raise outcome["error"]
        return outcome["value"]

    def join(self) -> None:
        """Wait until every queued write is done or given up."""
        self._jobs.join()

    def _run(self) -> None:
        while True:
            what, write, on_failure = self._jobs.get()
            try:
                self._apply(what, write, on_failure)
            finally:
                self._jobs.task_done()

    def _apply(self, what: str, write: Callable, on_failure: Callable | None) -> None:
        for attempt in range(1, self._attempts + 1):
            try:
                message = write(self.client)
            except Exception as exc:
                if attempt < self._attempts:
                    self._log(f"GraphDB: could not {what} ({exc}); retrying in {self._retry_seconds:g} s.",
                              "yellow")
                    time.sleep(self._retry_seconds)
                    continue
                self._log(f"GraphDB: gave up trying to {what} after {attempt} attempts: {exc}", "red")
                if on_failure is not None:
                    on_failure()
                return
            if message:
                self._log(message, "cyan")
            return
