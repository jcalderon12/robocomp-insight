from pydantic import BaseModel

class SimulationScene(BaseModel):
    gravity:float
    robot_velocity:float
    initial_robot_position:list[float]
    initial_robot_orientation:list[float]
    final_robot_position:list[float]
    final_robot_orientation:list[float]
    problem_position:list[float]
    problem_orientation:list[float]
    simulation_length:float
    num_of_repetitions:int