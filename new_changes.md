# Take Photos mission — handoff notes

Written for picking this up in a **fresh Claude Code session with zero prior context** (different machine, new chat). Read this whole file before doing anything. It replaces the old (stale, pre-revert) version of this changelog.

## What this is

Project "INSIGHT" (Shadow robot, RoboComp+CORTEX/DSR framework, Webots sim). Goal: extend the existing `follow_person` autopilot mission with a full chain:

```
follow_person (robot follows "person")
  -> bottle falls off robot -> "problem" node created (fake_castilla_la_mancha)
  -> "Search Problem Cause" mission (inner_simulator runs a DTW-matched causal search
     in a PyBullet sub-simulation to find what caused it and where)
  -> semantic validates the result, creates "bump" + "photograph_me" (+ has_intention),
     deletes "problem"
  -> "Take Photos" mission: robot approaches/orbits "bump", eventually should generate+train
     a YOLO/classifier agent ("concept_bump") from captured photos
```

This chain is now **structurally wired end-to-end** (mission creation, TARGET edges, affordance
handshake all work) but the robot's approach to `bump` is currently driving toward a garbage
position because of an active, unresolved data-corruption bug (see below). That bug is the
current blocker — fix it first.

## Architecture quick reference

- DSR graph shared across agents via CRDT/RTPS. Work graph = `G` (mission_controller) /
  `self.g` (single-graph agents like semantic, vision_sam, concept_robot).
- Affordance pattern (mirrors `follow_person`/`follow_me`/`concept_person`, reused generically
  for `Take Photos`/`photograph_me`): `TARGET` edge (robot -> target node) -> `has_intention`
  edge (target -> affordance node) -> affordance node has `aff_interacting` bool, set true by
  `mission_controller::on_startMission_clicked()` (fully generic, structural lookup, no
  hardcoded node names) once episodic_memory's handshake (`initialization_started` +
  `recording` on the mission node) completes.
- `concept_robot::follow_target()` is generic: reads whatever `TARGET` edge exists, reads
  `RT` translation from robot to that target, PID-drives toward it. Works for both `person`
  and `bump` unmodified.
- Each concept normally maintains its own live `RT robot->X`: `concept_person` for `person`,
  `concept_bottle` for `bottle`. **`bump` has no perception agent**, so `concept_robot::
  update_static_target_rt()` (new this session) computes `RT robot->bump` itself each cycle
  from `bump`'s static `problem_position` attribute + the live `room->robot` RT.

## Confirmed working (tested live this session)

- `semantic` (previously an empty scaffold) now does: watches `problem.cause_confirmed` (set
  by `inner_simulator::mark_cause_confirmed_in_dsr()` once its causal search finishes) ->
  creates `bump` (type `object`, carries `problem_position` mm + `pos_x`/`pos_y` copied from
  `problem`) -> creates `photograph_me` (type `affordance`, `aff_interacting=False`) ->
  creates `has_intention` edge `bump -> photograph_me` -> deletes `problem`.
- `mission_controller`: `Take Photos` mission gets created when `bump` exists (structural gate,
  same pattern as `follow_person` gating on `person` existing — not on any attribute payload),
  creates `TARGET robot->bump`. Handshake completes, `aff_interacting` gets set true, mission
  shows RUNNING, robot starts moving. All confirmed via live DSR viewer + logs.
- `vision_sam`: reworked away from the old "full image + YOLO bbox" idea (per user request) to
  simple RGBD capture: `<name>_rgb.jpg` (masked/cropped to SAM's segmentation) + `<name>_depth.npy`
  (raw float32 meters). Two capture modes in the UI now: SAM-segmented crop (`segment` button,
  click a point) and full raw frame (`Con bache`/`Sin bache` buttons, for a simple presence
  classifier — no SAM needed for that one). `sam2.1_l.pt` (large) confirmed working on this PC's
  RTX 3060 6GB, noticeably cleaner masks than `sam2.1_b.pt`. `view_capture.ipynb` (Jupyter, not
  a `.py`) exists to visually inspect a saved rgb+depth pair.
- Xbox controller + `python_xbox_controller` (robocomp-robolab) + `webots-bridge`'s existing
  `JoystickAdapter_sendData` (`do_joystick=true` hardcoded) gives manual joystick driving —
  no code needed, was just a Bluetooth pairing issue.

## Bugs found and fixed this session

- **`pydsr` Python `Edge()` constructor has reversed args**: `Edge(A, B, type, agent_id)` binds
  `A` to `.to()` and `B` to `.from()` (confirmed in `cortex/python-wrapper/python_api.cpp`,
  `"to"_a, "from"_a` in that order). This silently reversed `semantic`'s `has_intention` edge
  (bump->photograph_me came out as photograph_me->bump), which is why `aff_interacting` never
  flipped true for ages — `get_active_affordance_node()`'s `intention_edge.from() ==
  target_edge.to()` check never matched. Fixed. **Any new edge created via the Python `Edge()`
  constructor needs its first two args in `(to, from)` order, not `(from, to)`.** C++ side
  (`DSR::Edge` with explicit `.from()`/`.to()` setters) is unaffected/correct.
- `mission_controller::check_recording_handshake()` was requiring the `TARGET->has_intention->
  affordance` handshake to succeed for **every** mission type, including `Search Problem Cause`
  which has no affordance at all by design (resolved via simulation, not DSR affordance) — it
  would loop forever then get force-removed by the scheduler's 3-retry timeout, and that
  removal skipped `delete_mission_target_edge()`, poisoning the *next* mission's handshake too.
  Fixed both: added a `needs_affordance` type-check (only `follow_person`/`Take Photos`), and
  added the missing cleanup call to the `HANDSHAKE_TIMEOUT` event handler.
- `inner_simulator::convert_episodic_to_imu_history` crashed (`np.array` inhomogeneous shape)
  when a recorded `imu_accelerometer`/`imu_gyroscope` episodic event came back with **6 floats
  instead of 3**. A defensive fix (skip malformed-length entries, reuse last good value) was
  written, then **reverted at the user's explicit request** (wanted to go back to a known-good
  point) — the crash can still happen, just not currently guarded against.
- `bullshit_publisher`: `Create/Delete/Modify Attr` buttons existed in the `.ui` but were never
  wired up — implemented, using `G->runtime_checked_add_or_modify_attrib_local()` (the proper
  runtime/dynamic-attribute-name API — a raw `node.attrs()[name]=attr` write skips real
  timestamp/agent_id and doesn't propagate correctly over CRDT).

## 🔴 ACTIVE, UNRESOLVED BUG — pick up here

**Symptom:** robot drives toward `bump` but always in roughly the same direction/distance
regardless of where `bump` actually is — because `problem_position` (and therefore `bump`'s
position) comes out as a wildly implausible value (tens or hundreds of thousands of mm).

**What's been ruled out:**
- NOT a `concept_robot::update_static_target_rt()` math/scale bug — that function is correct;
  it's just being fed a bad `problem_position` to begin with.
- NOT the mm-vs-m unit mismatch that's a known, separate, pre-existing issue in this project
  (`concept_robot::auto_localization()` writes `room->robot` RT in meters while the rest of the
  project uses mm — see the `mm-m-unit-mismatch` persistent memory file). A fix attempting to
  multiply by 1000 in `inner_simulator::get_robot_positions_relative_to_problem()` was tried and
  made things **worse** (73788mm became 42,769,752mm), so **that hypothesis was wrong and the
  fix was reverted**. Do not reapply it without new evidence.

**Root cause, confirmed via direct evidence:** added a debug log line in
`get_robot_positions_relative_to_problem()` (still in the code, search for `[DEBUG] RAW
rt_translation`) and found the RAW value read from episodic_memory's recorded `room->robot`
`rt_translation` attribute is:
```
[1.0, 649384.0, -0.0, 320275.0, 0.0, 24162.0]
```
**6 floats, not 3.** This is the exact same corruption pattern as the (separately found, then
reverted) IMU crash bug above (`[0.0, 2000.0, -0.0, 2000.0, 9.0, 819000.0]`, also 6 floats).
**Same undiagnosed upstream bug, wider blast radius than first thought** — it corrupts
`rt_translation` too, not just IMU attributes. True root cause (why DSR/CRDT/episodic_memory
occasionally records 6 floats instead of 3 for a 3-element `vector<float>` attribute) was never
found — it's deep in `cortex` (the DSR core, a sibling repo), not in `robocomp-insight`.

**Where the log file actually is** (in case you need to check a fresh run — no ad-hoc
redirection needed, this already exists): `agents/inner_simulator/logs/specific_worker/
specific_worker_<timestamp>.log`, written by the existing `self.logger` (`Logger` class,
`inner_simulator/src/logger.py`). Only `self.logger.log(...)` calls land here — plain
`print()` (e.g. the DTW worker subprocess output) and Python `warnings` do not.

**Agreed next step, NOT YET applied:** add the same defensive guard used (then reverted) for
IMU, but for `rt_translation` reads in `get_robot_positions_relative_to_problem()`: if the value
isn't exactly length 3, discard that episodic event and fall back to the previous valid one in
history (or similar), instead of propagating the corrupt 6-float array into `problem_position`.
This does not fix the root cause, just stops it from poisoning `problem_position`. **User had
just said "¿Lo implemento así?" / agreed conceptually but implementation was not yet written
when this handoff was created — start there.**

## Other pending / deferred (lower priority than the bug above)

- `concept_bump` (the actual generated YOLO-detector agent) — **never implemented**. Currently
  `bump`/`photograph_me` are created directly by `semantic`, with no generated agent involved
  at all. `agent_generation` package (generator + templates) exists but its `generate_agent()`
  call site in `inner_simulator` is commented out, and the template doesn't create any DSR
  nodes on startup. This is intentionally staged for later — get the movement/photo-taking
  right manually first.
- Orbit/circling movement around `bump`: not implemented. A first attempt (`orbit_target()`)
  was built and then fully reverted earlier (caused accelerometer anomalies, never root-caused —
  possibly related to the same 6-float corruption bug above, unconfirmed). Currently
  `concept_robot` only *approaches* `bump` via the generic `follow_target()`, doesn't orbit.
  Do not re-attempt orbit until the position-corruption bug above is actually fixed — testing
  movement logic against garbage input isn't useful.
- No logic yet for when to stop approaching / take a photo / end the `Take Photos` mission.
- `bullshit_publisher`: attribute type/value UI fields (currently Create/Modify Attr only
  support a hardcoded bool placeholder) — explicitly deferred by the user ("para el final").
- `vision_sam`'s `con_bache`/`sin_bache` manual dataset (in `segmented_objects/bump_dataset/`,
  35/35 balanced) is for prototyping a simple presence classifier — separate concern from
  `concept_bump`'s eventual YOLO detector, which will need the full-frame+bbox format instead
  (the format explicitly dropped earlier this session per user request, "obviando la idea
  original de formato" — would need to be reinstated for that specific future purpose).

## Environment notes

- `caveman` Code plugin is active for this session (terse-response mode) — irrelevant to a
  fresh session unless the user re-enables it there too.
- A GitHub PAT got accidentally pasted into a terminal session mid-conversation and should have
  been revoked by the user — not a code issue, just flagging in case it wasn't done.
- Persistent memory file `mm_m_unit_mismatch.md` (Claude's own memory system, separate from this
  repo) has the fuller history of the meters-vs-millimeters issue across this codebase.
