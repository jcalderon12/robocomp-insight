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
    def modify_robot(self):
        "Modify the robot in the simulation scene"
        pass

    @abstractmethod
    def disable_robot_wheel(self, wheel_name):
        "Disable the robot wheel in the simulation scene"
        pass
    
        
    