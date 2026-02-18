from abc import ABC, abstractmethod

class ApplyAction(ABC):
    
    @abstractmethod
    def render_action_variables():
        pass

    @abstractmethod
    def render_action_call():
        pass

