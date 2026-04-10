from pydantic import BaseModel

class SimulationScene(BaseModel):
    gravity:float
    initial_robot_position:list[float]
    initial_robot_orientation:list[float]
    bottle_position:list[float]
    bottle_orientation:list[float]
    problem_position:list[float]
    problem_orientation:list[float]
    simulation_length:float
    num_of_repetitions:int
    list_of_target_velocities:dict