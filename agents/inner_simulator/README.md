# inner_simulator

## Overview

`inner_simulator` is a RoboComp agent that maintains a PyBullet physics simulation mirroring the real robot. It publishes synthetic IMU data to the DSR graph and, when the robot encounters a problem, launches parallel simulations of candidate *causes* (disturbances such as bumps or wheel failures) to identify which one best explains the observed sensor readings.

The project is organised around four subsystems:

1. **Runtime simulation** — `SpecificWorker` keeps a live PyBullet scene synchronised with DSR and records real IMU data.
2. **Cause evaluation** — `CausesSimulator` runs headless PyBullet instances (one per candidate cause) and returns simulated IMU traces for comparison.
3. **Cause generation** — a code-generation pipeline under `src/cause_gen/` that turns a JSON cause definition into a ready-to-use Python `Cause` subclass.
4. **Hypothesis integration** — modules that connect the simulator to the semantic agent: `hypothesis_compiler.py` (LLM blueprint → cause payloads), `episode_scene.py` (scene poses from the episodic recording), and `verdict.py` (contrastive accept/reject decision against a nominal baseline).

## Hypothesis-driven flow (semantic ↔ inner_simulator)

When the semantic agent detects an unexplained change it publishes a hypotheses
batch (grounded in the shared catalog `etc/intervention_catalog.json` at the repo
root) and exposes its path as the `hypotheses_filepath` attribute of the
`unexplained` intention node in the **work** DSR graph. On the next
"Search Problem Cause" mission:

1. `SpecificWorker` (IDLE) picks the batch up, compiles it with
   `hypothesis_compiler.compile_batch` into cause payloads (a nominal `none`
   cause is always prepended as the null hypothesis) and reconstructs the scene
   poses from the episodic keyframe (`episode_scene.extract_scene_poses`).
2. Each cause runs headless in its own subprocess (`CausesSimulator`,
   `--gui` optional for debugging).
3. `verdict.build_verdict` accepts a hypothesis iff one of its repetitions
   reproduces the symbolic effect (bottle off the tray) **and** explains the
   real IMU trace over the anomaly window better than the nominal run by a
   configurable margin. The verdict JSON is written to `logs/verdicts/` and its
   path is published back as the `verdict_filepath` attribute of the
   `unexplained` node, where the semantic agent ingests it.

Without a published batch the agent falls back to the static `src/causes.json`
list (legacy one-shot behavior).

> **Operational order**: the episodic recording is flushed to its file when the
> mission finishes, so "Follow Person" must be **completed** before launching
> "Search Problem Cause". If the recording cannot be indexed the episode is
> skipped with a diagnostic (an empty file means the mission was not completed;
> decimal commas mean the recorder agent ran under a non-C numeric locale).

## Configuration

| File | Purpose |
|---|---|
| `etc/config` | Agent ID, DSR flags, compute period. |
| `src/sim_scene.json` | `SimulationScene` parameters (gravity, poses, velocity, repetitions). |
| `src/causes.json` | Fallback list of cause payloads when no hypotheses batch is published. |
| `../../etc/intervention_catalog.json` | Shared catalog of executable interventions and URDF assets (contract with the semantic agent). |

## Starting the component

```bash
bin/inner_simulator etc/config
```

## Classes

> Abstract base classes are documented as contracts. Concrete cause implementations under `src/causes/implementations/` are intentionally omitted.

### `SpecificWorker` — `src/specificworker.py`

Main RoboComp agent. Runs a PyBullet scene, reads robot velocities from DSR, drives the simulated robot accordingly, and publishes synthetic IMU readings back to DSR. When a problem is detected (the `follow_me` affordance becomes inactive), it serialises the current scene, loads the cause list from `src/causes.json`, spawns a `CausesSimulator` subprocess per cause, collects their IMU histories, and compares them against the real recording to find a match.

- **Parameters:** `proxy_map`, `configData` (dict with `Period.Compute`), `startup_check` (bool).
- **Output:** Synthetic IMU node (`imu_sintetic`) in the DSR graph; console report of matched causes.

### `SimulationScene` — `src/simulation_scene.py`

Pydantic model that holds every parameter needed to configure a simulation run.

- **Fields:**
  - `gravity: float` — vertical gravity component (m/s²).
  - `robot_velocity: float` — forward speed set-point (m/s).
  - `initial_robot_position: list[float]` — starting `[x, y, z]`.
  - `initial_robot_orientation: list[float]` — starting quaternion `[x, y, z, w]`.
  - `final_robot_position: list[float]` — pose when the problem was detected.
  - `final_robot_orientation: list[float]` — orientation when the problem was detected.
  - `problem_position: list[float]` — position at the moment of failure.
  - `problem_orientation: list[float]` — orientation at the moment of failure.
  - `simulation_length: float` — duration of each run (seconds).
  - `num_of_repetitions: int` — how many times the scenario is repeated.
- **Output:** Serialisable JSON consumed by `CausesSimulator`.

### `IMU` — `src/pybullet_imu.py`

Simulates an IMU sensor attached to a PyBullet body using the model $\tilde{a} = R_{W}^{IMU}(a + g) + b + w$.

- **Parameters:** `body_id` (PyBullet body handle), `dt` (simulation time-step).
- **Output:** `get_measurement()` → `(accelerometer: np.ndarray[3], gyroscope: np.ndarray[3])`.

### `CausesSimulator` — `src/causes_simulator.py`

Headless simulation runner. Initialises its own PyBullet instance, loads a `SimulationScene`, validates the cause JSON against a dynamically-built discriminated Pydantic union of all `Cause` subclasses found in `src/causes/implementations/`, and executes `num_of_repetitions` simulation runs collecting IMU traces.

- **Parameters:** `cause` (JSON string), `simulation_scene` (path to scene JSON), `pipe` (file-descriptor for IPC), `real_time` (bool, optional).
- **Output:** JSON array of IMU histories written to the pipe via `send_history_to_parent()`.

### `Cause` *(abstract)* — `src/causes/cause.py`

Base interface for all simulation disturbances. Every cause must implement two hooks:

| Method | Called | Purpose |
|---|---|---|
| `apply(self, engine)` | Once before each run | Prepare the scene (e.g., spawn an obstacle). |
| `apply_compute(self, engine)` | Every simulation step | Apply time-dependent behaviour (e.g., disable a wheel at a given instant). |

Both methods receive an `Engine` instance to interact with the simulator.

### `Engine` *(abstract)* — `src/engines/engine.py`

Simulator-agnostic API that causes use to modify the world.

| Method | Description |
|---|---|
| `instantiate_body(body_file, body_position)` | Spawn a URDF body at the given position. |
| `get_simulation_time()` | Current simulation time (s). |
| `disable_robot_wheel(wheel_name)` | Disable a wheel by simplified name (`FL`, `FR`, `BL`, `BR`). |
| `get_simulation_length()` | Total configured run duration (s). |

### `EnginePybullet` — `src/engines/engine_pybullet.py`

Concrete `Engine` implementation that delegates every call to a `CausesSimulator` instance.

- **Parameters:** `sim_instance` (`CausesSimulator`).
- **Output:** Forwards all operations to the PyBullet-backed simulator.

### `DefinedCause` — `src/cause_gen/defined_cause.py`

Pydantic model and Jinja2-based code generator. Reads a JSON cause definition containing instance generators and actions, and renders a complete Python `Cause` subclass file.

- **Fields:**
  - `name: str` — cause identifier (used as the Pydantic `Literal` discriminator).
  - `description: str` — docstring for the generated class.
  - `instance_generators: list[InstanceGeneratorUnion]` — strategies that produce runtime values.
  - `apply_actions: list[ApplyActionUnion]` — actions executed in `apply()`.
  - `apply_compute_actions: list[ApplyActionUnion]` — actions executed in `apply_compute()`.
- **Output:** `render_cause()` → Python source code string of the generated `Cause` subclass. Can be run as a CLI via `python defined_cause.py -f <json> [-o <output>]`.

### `ApplyAction` *(abstract)* — `src/cause_gen/apply_action/apply_action.py`

Contract for code-generation actions used by `DefinedCause`. Each implementation renders fragments of Python code that will be embedded in the generated cause class.

| Method | Returns |
|---|---|
| `render_action_variables()` | List of field/variable declarations (strings). |
| `render_action_call()` | List of executable statements (strings). |

Example implementations: `ApplyActionInstantiateBody` (spawns a body at a generated position), `ApplyActionStopRobotWheel` (disables a wheel after a time threshold).

### `InstanceGenerator` *(abstract)* — `src/cause_gen/instance_generator/instance_generator.py`

Contract for code-generation strategies that produce runtime values (coordinates, random numbers, etc.) embedded into generated cause classes.

| Method | Returns |
|---|---|
| `render_generate_instance()` | A string containing a complete Python method definition. |

Example implementations: `InstanceRandomRangeCoordinates` (random 3-D point within a bounding box), `InstanceSimpleRandom` (single random float), `InstanceNone` (no-op placeholder).

## Architecture notes

- **Cause discovery** — At import time, `causes_simulator.py` scans `src/causes/implementations/`, collects every `Cause` subclass, and builds a Pydantic discriminated union keyed on the `name` literal. This allows cause JSON payloads to be validated and deserialised automatically.
- **IPC model** — `SpecificWorker` spawns each `CausesSimulator` as a subprocess, passing a write pipe file-descriptor. The child writes its JSON history to the pipe; the parent reads and deserialises it after the child exits.
- **Cause generation** — Running `python src/cause_gen/defined_cause.py -f definition.json` produces a self-contained `.py` file that can be dropped into `src/causes/implementations/` and will be picked up automatically on the next run.

## License

This project follows the RoboComp licensing model (GPLv3). See individual source file headers for details.
