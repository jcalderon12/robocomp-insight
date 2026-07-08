from abc import ABC, abstractmethod

class Engine(ABC):

    @abstractmethod
    def instantiate_body(self, body_file, body_position):
        "Instantiate a phisyc body in the simulation scene"
        pass
    
    @abstractmethod
    def get_simulation_time(self):
        "Get the current simulation time"
        pass

    @abstractmethod
    def disable_robot_wheel(self, wheel_name):
        "Disable the robot wheel in the simulation scene"
        pass

    @abstractmethod
    def get_simulation_length(self):
        "Get the whole simulation length"
        pass

    @abstractmethod
    def apply_external_force(self, target_name, force_vector):
        "Apply an external force vector (world frame, Newtons) to a named body for the next simulation step"
        pass

    @abstractmethod
    def set_lateral_friction(self, target_name, value):
        "Set the lateral friction coefficient of a named body"
        pass
        
    