from engine import Engine
from causes_simulator import CausesSimulator

class EnginePybullet(Engine):
    
    def __init__(self, sim_intance):
        self.sim_instance:CausesSimulator = sim_intance # Simultion class
        self.p = sim_intance.p  # Pybullet instance

    def instantiate_body(self, body_file, body_position):
        self.p.loadURDF(body_file, body_position)
        
    def get_simulation_time(self):
        return 
    
    def disable_robot_wheel(self, wheel_name):
        self.sim_instance.set_robot_wheel_moving(wheel_name, False)