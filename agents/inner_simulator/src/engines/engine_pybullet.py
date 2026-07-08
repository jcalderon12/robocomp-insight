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

    def apply_external_force(self, target_name, force_vector):
        self.sim_instance.apply_external_force(target_name, force_vector)

    def set_lateral_friction(self, target_name, value):
        self.sim_instance.set_lateral_friction(target_name, value)