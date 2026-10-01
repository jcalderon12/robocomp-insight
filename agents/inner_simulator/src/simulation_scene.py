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