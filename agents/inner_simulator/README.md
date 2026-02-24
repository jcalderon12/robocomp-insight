# inner_simulator

## Overview
`inner_simulator` provides a PyBullet-based simulation environment and a small framework to define and run "causes" (simulation events or disturbances). It includes an engine abstraction, Pydantic models for scene configuration, IMU simulation, and a worker that integrates the simulation into the RoboComp stack.

## Configuration parameters
The component uses a configuration file (example located at `etc/config`). The main runtime parameters are provided through the agent `config` and the `SimulationScene` JSON files (see `src/simulation_scene.py` for fields).

## Starting the component
To run locally without modifying the repository config:

```bash
cd /path/to/inner_simulator
cp etc/config config
# edit `config` as needed
bin/inner_simulator config
```

## Classes
This section lists the main classes in the project, with a short description, constructor / parameters and produced outputs. Abstract base classes are documented as contracts that concrete implementations must satisfy. Concrete cause implementations under `src/causes/implementations` are intentionally omitted from this document.

- `ApplyAction` (abstract):
	- Description: Contract for an action that modifies the simulated world as part of a cause. Implementations are code-generation helpers used when defining causes.
	- Methods / parameters: `render_action_variables()` and `render_action_call()` — abstract render methods that subclasses must implement (expected to return textual/code fragments used when generating/appling actions).
	- Output: No runtime side-effect by itself; subclass implementations typically return strings used in cause-generation or perform action-specific calls.

- `InstanceGenerator` (abstract):
	- Description: Contract for strategies that generate instances (positions, parameters, etc.) used by causes. Used by the cause generator subsystem.
	- Methods / parameters: `render_generate_instance()` — abstract method to produce an instance representation; concrete subclasses implement generation logic.
	- Output: Subclass-defined instance representation (commonly serialized or used to instantiate objects in generated cause code).

- `Cause` (abstract) — `src/causes/cause.py`:
	- Description: Base interface for all simulation causes. A cause represents a disturbance or scenario modification applied to the simulation.
	- Methods / parameters: Implementations must provide:
		- `apply(self, engine)`: invoked once before a series of simulation steps to prepare the scene.
		- `apply_compute(self, engine)`: invoked during the simulation compute loop to apply time-dependent behavior.
		Both methods receive an `engine` instance (an Engine implementation) to interact with the simulator.
	- Output: Causes typically modify the simulation state via the provided `engine` (e.g., instantiate bodies, disable wheels).

- `CausesSimulator` — `src/causes_simulator.py`:
	- Description: Orchestrates PyBullet simulation runs for a given cause and scene. Loads causes dynamically, manages simulation loop, collects IMU data, and communicates results back to the parent process.
	- Constructor parameters: `cause` (JSON string defining cause values), `simulation_scene` (path to a `SimulationScene` JSON), `pipe` (pipe identifier to send results), `real_time` (bool; optional).
	- Key responsibilities / outputs:
		- Loads a `SimulationScene` (Pydantic model) and applies simulation parameters.
		- Initializes PyBullet, robot model, IMU sensor, and engine wrapper (`EnginePybullet`).
		- Runs simulations (`simulate`, `doSimulations`) collecting IMU traces and storing them in `self.historical`.
		- Sends recorded history as JSON through the provided pipe via `send_history_to_parent()`.
		- Dynamically discovers cause subclasses (plugin loading) from `src/causes/implementations` and validates them against the `Cause` base class.

- `Engine` (abstract) — `src/engines/engine.py`:
	- Description: Minimal engine API that cause implementations use to interact with the underlying simulator.
	- Methods / parameters (abstract):
		- `instantiate_body(self, body_file, body_position)` — spawn a body in the scene.
		- `get_simulation_time(self)` — return current simulation time.
		- `disable_robot_wheel(self, wheel_name)` — disable a named wheel.
		- `get_simulation_length(self)` — return configured simulation length.
	- Output: None directly; implementations delegate to the concrete simulator (e.g., `EnginePybullet`).

- `EnginePybullet` — `src/engines/engine_pybullet.py`:
	- Description: Concrete `Engine` implementation that delegates calls to `CausesSimulator` to operate on a PyBullet-backed scene.
	- Constructor parameters: `sim_instance` (the `CausesSimulator` instance to operate on).
	- Methods / outputs: Implements the `Engine` contract by forwarding `instantiate_body`, `get_simulation_time`, `disable_robot_wheel`, and `get_simulation_length` to the simulator instance.

- `SimulationScene` (Pydantic model) — `src/simulation_scene.py`:
	- Description: Typed model describing static simulation parameters used to configure a run.
	- Fields / parameters:
		- `gravity: float`
		- `robot_velocity: float`
		- `initial_robot_position: list[float]`
		- `initial_robot_orientation: list[float]`
		- `final_robot_position: list[float]`
		- `final_robot_orientation: list[float]`
		- `problem_position: list[float]`
		- `problem_orientation: list[float]`
		- `simulation_length: float`
		- `num_of_repetitions: int`
	- Output: Used by `CausesSimulator` to configure gravity, initial pose, and loop length.

- `IMU` — `src/pybullet_imu.py`:
	- Description: Simple IMU simulator that approximates accelerometer and angular velocity readings from PyBullet state.
	- Constructor parameters: `body_id` (PyBullet body id of the robot), `dt` (simulation timestep).
	- Key method: `get_measurement()` — returns `(accelerometer, angular_velocity)` as NumPy arrays. The sensor model includes bias and white noise terms.

- `GenericWorker` — `generated/genericworker.py`:
	- Description: Base GUI/agent worker (generated). Initializes UI, a `DSRGraph` instance, and a compute timer used by `SpecificWorker`.
	- Constructor parameters: `(mprx, configData)` — generated scaffolding expects proxy map and config data.
	- Provided features: `setPeriod()` and `killYourSelf()` slots, UI wiring and a `Period` timer.

- `SpecificWorker` — `src/specificworker.py`:
	- Description: Application worker that embeds the simulation in the RoboComp stack. It creates a PyBullet scene for debugging, interacts with DSR signals, publishes IMU data, and uses `CausesSimulator` to run cause-driven experiments.
	- Constructor parameters: `proxy_map` (proxy map), `configData` (configuration dictionary), `startup_check` (bool).
	- Key behavior / outputs:
		- Sets up PyBullet world, loads robot and models, initializes `IMU` and `SimulationScene` defaults.
		- Provides methods to record IMU traces and write debug `sim_scene.json`.
		- Uses `CausesSimulator` to execute cause-driven simulation experiments when required.

## Notes and development pointers
- The cause discovery mechanism in `src/causes_simulator.py` dynamically imports modules from `src/causes/implementations` and builds a discriminated Pydantic union to validate cause JSON payloads.
- The cause generation subsystem uses `ApplyAction` and `InstanceGenerator` abstractions along with additional helper modules under `src/cause_gen/` to produce or serialize new causes. The abstract base classes live in `src/cause_gen/apply_action/apply_action.py` and `src/cause_gen/instance_generator/instance_generator.py`.

## Contact / License
This project follows the licensing pattern used in RoboComp examples. See source headers for license information.

--
Documentation generated to describe the main classes and how they interact. For more details or to include per-class code examples, tell me which classes you want expanded.
