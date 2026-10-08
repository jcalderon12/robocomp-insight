"""Offline test: the LLM in front of the published batch (agents/semantic/src/hypothesis_generator.py).

No LLM is called: a scripted one answers. Checks:
  * the prompt of the 12:29 fall of 2026-09-24 gives the episode with every anchor a hypothesis
    may take (and not the system's reaction after the fall, neither as evidence nor as anchor), the
    closed list of mechanisms from the TBox with their kinds (no magnitude) and costs, the robot's
    self-model and the budget, and no number of the grounding (catalog assets, ranges, windows);
  * an answer that is not a JSON object, or breaks the schema, is sent back with its problems and
    asked again; a failure to reach the LLM is asked again as it was; the number of attempts and
    each attempt's outcome stay in the batch;
  * when no answer passes in max_attempts, the batch is published with status `error`, no
    hypotheses and the problems, and the simulator's compiler only runs the nominal;
  * the batch is saved with its prompt and the whole conversation, and the six ideas of the real
    12:29 batch, answered by the scripted LLM, give the batch of contract 3, section 4.4.

The recordings are not in git (agents/mission_controller/recorded_missions/), nor is experiments/:
both travel in the hand-over package.

Run from the repo root:  python3 tests/offline/test_hypothesis_generator.py
"""
import json
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "agents" / "semantic"))
sys.path.insert(0, str(REPO / "agents" / "inner_simulator" / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from hypothesis_compiler import compile_batch  # noqa: E402
from src.explanation_context import NEUTRAL_IDS  # noqa: E402
from src.hypothesis_generator import build_prompt, generate_batch, read_self_model  # noqa: E402
from src.hypothesis_pipeline import MECHANISM_ORDER  # noqa: E402
from test_episode_contrast import episodes_2409  # noqa: E402
from test_hypothesis_pipeline import EXPECTED_1229, six_ideas_1229  # noqa: E402


class ScriptedLLM:
    """Answers in turn with the scripted replies; an exception is raised as the LLM failing."""

    def __init__(self, *replies):
        self.replies = list(replies)
        self.seen = []

    def __call__(self, messages):
        self.seen.append([dict(m) for m in messages])
        reply = self.replies.pop(0)
        if isinstance(reply, Exception):
            raise reply
        return reply, {"eval_count": 1}


def generate(episode, llm, tmp, **options):
    return generate_batch(episode, llm, model="scripted", output_dir=Path(tmp), case_id="case", **options)


def check_prompt_1229(episode):
    prompt = build_prompt(episode, read_self_model(), budget=6)
    for segment in episode["segments"]:
        assert f"- {segment['id']}:" in prompt, segment["id"]
    for interval in episode["intervals"]:                                # shown with its neutral id
        offered = f"- {NEUTRAL_IDS.get(interval['id'], interval['id'])} = [" in prompt
        assert offered == (interval["start_s"] < episode["time"]["t_obs_s"]), interval["id"]
    assert "Interval_reaction" not in prompt
    braking = next(e for e in episode["evidence"] if e.get("is_system_reaction"))
    assert f"{braking['value']:.3f}".rstrip("0") not in prompt, braking
    for mechanism_id in MECHANISM_ORDER + ("new",):
        assert f"- {mechanism_id}:" in prompt, mechanism_id
    # Kinds only, with what each one costs: the magnitudes are the memory's.
    assert "Parameters: shape: bump | cable." in prompt and "Cost: a bump, 4 simulations" in prompt
    mechanisms = prompt[prompt.index("## The mechanisms"):prompt.index("## The robot")]
    for magnitude in ("size:", "height:", "strength:", "slipperiness:"):
        assert magnitude not in mechanisms, magnitude
    assert "Shadow uses a differential-drive mobile base" in prompt          # the self-model, once
    assert prompt.count("## The robot (its self-model)") == 1
    assert "up to 6 simulations" in prompt
    # Without a budget (production since 07/10) the LLM reads neither a budget nor costs.
    unlimited = build_prompt(episode, read_self_model())
    assert "every one that survives and can be simulated is simulated" in unlimited
    assert "simulations are run" not in unlimited and "Cost:" not in unlimited
    assert "Parameters: shape: bump | cable." in unlimited and "Simulable: yes." in unlimited
    assert "bottle_reacquired = no." in prompt and "pitch_rate_peak = 0.948 rad/s at 12.928 s" in prompt
    # The grounding is the memory's: no catalog asset, range nor window reaches the LLM.
    for term in ("bump_", "cylinder_bump", "position_range", "force_range", "activation_window", "start_fraction",
                 "lateral_friction_range"):
        assert term not in prompt, term


def valid_answer():
    return json.dumps(six_ideas_1229())


def check_retries_with_feedback(episode):
    gremlins = json.dumps({"hypotheses": [{"mechanism": "gremlins", "interval": "Interval_fall"}]})
    llm = ScriptedLLM(ConnectionError("cloud unreachable"), "It was surely a bump.", gremlins,
                      "```json\n" + valid_answer() + "\n```")
    with tempfile.TemporaryDirectory() as tmp:
        result = generate(episode, llm, tmp, max_attempts=4)
        assert result.ok and result.batch["status"] == "success", result.batch["errors"]
        assert result.batch["attempts"] == 4 and result.batch["model"] == "scripted"
        assert [a["status"] for a in result.batch["generation_metrics"]["attempts"]] == [
            "llm_error", "refused", "refused", "success"]
        # A failure to reach the LLM is asked again as it was; a refused answer, with its problems.
        assert llm.seen[0] == llm.seen[1] and len(llm.seen[1]) == 1
        assert "not a JSON object" in llm.seen[2][-1]["content"], llm.seen[2][-1]
        assert "unknown mechanism 'gremlins'" in llm.seen[3][-1]["content"], llm.seen[3][-1]
        assert llm.seen[3][-2] == {"role": "assistant", "content": gremlins}
        # Saved: the batch, its prompt and the whole conversation.
        saved = json.loads(result.batch_path.read_text(encoding="utf-8"))
        assert saved == result.batch
        assert Path(saved["prompt_path"]).resolve() == result.prompt_path.resolve()     # outside the repo: absolute
        assert result.prompt_path.read_text(encoding="utf-8").strip() == llm.seen[0][0]["content"].strip()
        transcript = json.loads(result.transcript_path.read_text(encoding="utf-8"))
        assert [m["role"] for m in transcript["messages"]] == ["user", "assistant", "user", "assistant", "user",
                                                                "assistant"]
    statuses = {h["hypothesis_id"]: h["status"] for h in result.batch["hypotheses"]}
    assert statuses == {hid: status for hid, (status, _) in EXPECTED_1229.items()}, statuses
    assert result.batch["arm"] == "llm_full"
    assert result.batch["hypotheses"][1]["llm_title"] == "External Physical Contact"


def check_gives_up(episode):
    llm = ScriptedLLM("no", json.dumps({"hypotheses": [{"mechanism": "bottle_push", "interval": "Interval_fall",
                                                        "qualitative_parameters": {"strength": 12}}]}),
                      ConnectionError("cloud unreachable"))
    with tempfile.TemporaryDirectory() as tmp:
        result = generate(episode, llm, tmp)
        assert not result.ok and result.batch["status"] == "error"
        assert result.batch["hypotheses"] == [] and result.batch["attempts"] == 3
        assert len(result.batch["errors"]) == 3, result.batch["errors"]
        assert "numbers are not allowed" in result.batch["errors"][1], result.batch["errors"]
        assert result.batch["budget"]["used"] == 0 and result.batch_path.exists()
        compiled = compile_batch(result.batch)
        assert len(compiled["entries"]) == 1 and not compiled["skipped"]           # only the nominal run


def check_budget_config():
    """Production simulates everything that survives (Budget = "all", 07/10); a number is a budget k."""
    from src.hypothesis_config import ConfigError, _as_budget
    assert [_as_budget(v) for v in (None, "all", '"all"', 6, "3", 0)] == [None, None, None, 6, 3, 0]
    try:
        _as_budget("six")
    except ConfigError:
        pass
    else:
        raise AssertionError("a budget that is neither \"all\" nor a number is refused")
    config = (REPO / "agents" / "semantic" / "etc" / "config").read_text(encoding="utf-8")
    line = next(l for l in config.splitlines() if l.startswith("hypothesisGenerator.Budget"))
    assert _as_budget(line.split("=", 1)[1].strip()) is None, line
    print("OK check_budget_config")


def main():
    episode = episodes_2409()["122924"]
    for check in (check_prompt_1229, check_retries_with_feedback, check_gives_up):
        check(episode)
        print(f"OK {check.__name__}")
    check_budget_config()
    print("Hypothesis generator: all checks passed")


if __name__ == "__main__":
    main()
