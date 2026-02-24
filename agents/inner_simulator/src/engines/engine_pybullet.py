from engines.engine import Engine
from uuid import uuid4
import pybullet as p

class EnginePybullet(Engine):
    
    def __init__(self, sim_intance):
        self.sim_instance:CausesSimulator = sim_intance # Simultion class

    def instantiate_body(self, body_file, body_position):
        self.sim_instance.instantiate_body(body_file, body_position)
        
    def get_simulation_time(self):
        return self.sim_instance.get_simulation_time()
    
    def disable_robot_wheel(self, wheel_name):
        self.sim_instance.set_robot_wheel_moving(wheel_name, False)
    
    def get_simulation_length(self):
        return self.sim_instance.get_simulation_length()