from abc import ABC, abstractmethod

class InstanceGenerator(ABC):

    @abstractmethod
    def render_generate_instance():
        """
        Render the code of the instance generator.
        """
        pass
    
    