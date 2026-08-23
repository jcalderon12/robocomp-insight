# New mission: search cause -> take photos

Changelog for the "Take Photos" mission built on top of follow_person/search_problem_cause. Covers everything since commit `4ada91a` (first commit of this work) plus uncommitted changes. Quick-check list, not full docs.

## inner_simulator
- Extracts the winning bump position from the DTW cause-matching, writes it as `problem_position` on the `problem` node.
- Fixed `check_for_problems()`: matched mission names against the wrong string (mission type instead of the actual generated name `search_cause_attempt_N`/`follow_person_attempt_N`) - search_problem_cause never used to start.

## vision_sam
- New 3D->2D projection (`project_point_3d_to_2d`), real zed camera offset/rotation (from Shadow.proto) + live room->robot RT. Triggers SAM capture when `problem_position` appears on a node (hooks both node creation and attr update signals).
- `save_segmented_object` now saves full unmasked image + YOLO `.txt` label (bbox from SAM mask), not a masked crop.
- Fixed hover/click pixel coordinate scaling bug (hover wasn't scaled to pixmap resolution).

## agent_generation (new package, was duplicated in inner_simulator + bullshit_publisher)
- Consolidated `agent_generator.py` + templates into one place.
- New `yolo_dataset.py` / `yolo_trainer.py`.
- Generated agents named `concept_<name>` consistently (folder + component; used to mismatch).

## mission_controller
- New "Take Photos" mission, triggered when `problem_position` appears on `problem` (also deletes `problem`, so a future incident can be detected again).
- Fixed two mission-type casing bugs (`search_problem_cause` vs `Search Problem Cause`, dropdown `Follow Person` vs internal `follow_person`).
- Fixed handshake: activating an affordance was a single, un-retried attempt - now retries every cycle until it succeeds.
- Autopilot no longer stops when "Search Problem Cause" completes (was cutting the chain before "Take Photos" launched) - only on "follow_person"/"Take Photos" completing.
- Generalized affordance activation/deactivation/completion-check to work off TARGET->has_intention structure, not hardcoded "follow_me" - "photograph_me" works for free.
- Fixed: previous work-graph TARGET edge was never deleted, would've made two coexisting TARGETs ambiguous.

## concept_robot
- New `orbit_target()`: circles the current TARGET at fixed radius (sibling to `follow_target`). `compute()` picks orbit vs follow based on active affordance name.
- Refactored `follow_target` into reusable pieces (`get_target_relative_position`, `compute_approach_velocities`) so orbit reuses the same PID.
- Generalized `queck_affordance_active` (same structural lookup as mission_controller).

## episodic_memory
- Added `print_extra_info` flag (default false) to quiet per-signal console spam.

## fake_castilla_la_mancha / fake_malaga
- Fixed startup crash: `print_dsr_signals` was initialized after connecting DSR signals; an early signal could crash the process.

## Still open
- Who creates `bump`/`photograph_me` + generates/launches `concept_bump`: decided as `inner_simulator`, not implemented yet.
- `concept_bump` keeping a live `robot->bump` RT (mirroring `concept_person`'s `robot->person`): not implemented yet.
- No logic yet for when to stop orbiting / take a photo / end the mission.
