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
    list_of_target_rot_speeds:dict = {} # {"timestamp": [...], "rot_speed": [...]} in rad/s; empty keeps the robot from turning.
    observed_effect_time:Optional[float] = None # Seconds at which the bottle was observed leaving the robot, if it was.
    episode_length:Optional[float] = None # Length of the whole recording, before cutting the horizon at the observed effect.
    clock_rate:float = 1.0 # Seconds of the recorded physics per second of the recording's clock (Webots runs slower than the wall clock the episode is stamped with); 1 replays on the recording's clock.
