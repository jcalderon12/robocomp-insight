from typing import Optional

from pydantic import BaseModel

class SimulationScene(BaseModel):
    """ Data model representing the configuration of a simulation scene."""
    gravity:float # Gravity value for the simulation environment (m/s^2)
    initial_robot_position:list[float] # Initial position of the robot in millimeters.
    initial_robot_orientation:list[float] # Initial orientation of the robot.
    bottle_position:list[float] # Initial position of the bottle in millimeters.
    bottle_orientation:list[float] # Initial orientation of the bottle.
    problem_position:list[float] # Initial position of the problem in millimeters.
    problem_orientation:list[float] # Initial orientation of the problem.
    simulation_length:float # Length of the simulation.
    num_of_repetitions:int # Number of times to run the simulation.
    list_of_target_velocities:dict # List of target velocities for the robot.
    list_of_target_rot_speeds:dict = {} # {"timestamp": [...], "rot_speed": [...]} in rad/s, positive counterclockwise; empty keeps the robot from turning.
    observed_effect_time:Optional[float] = None # Seconds at which the bottle was observed leaving the robot, if it was.
    episode_length:Optional[float] = None # Length of the whole recording, before cutting the horizon at the observed effect.
    clock_rate:float = 1.0 # Seconds of the recorded physics per second of the recording's clock (Webots runs slower than the wall clock the episode is stamped with); 1 replays on the recording's clock.
    physics_dt:Optional[float] = None # PyBullet step (s); None keeps the simulator's default.
    start_time_s:float = 0.0 # Instant of the recording the simulation starts at, from the recorded state (episode_scene.SEED_LEAD_S before the anomaly interval); 0, the start of the recording.
    initial_speed:float = 0.0 # Forward speed (m/s) of the recorded robot at start_time_s.
    anomaly_interval:Optional[list[float]] = None # [start, end] s of the episode's anomaly interval (end = t_obs), which the verdict compares on.
    anomaly_footprint:Optional[dict] = None # {x: [min, max], y: [min, max]} m the robot covered during the anomaly interval (its path dilated by its reach).
    world_time_step_s:Optional[float] = None # Physics step (s) of the recorded Webots world: the span each recorded accelerometer sample averages over, and the step Webots' soft contacts are defined with. None: Webots' default, 32 ms.
