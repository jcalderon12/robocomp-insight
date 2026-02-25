from abc import ABC, abstractmethod

class Cause(ABC):

    @abstractmethod
    def apply(self, engine):
        """
        Apply an instance of the cause.
        """
        pass

    @abstractmethod
    def apply_compute(self, engine):
        """
        Apply an instance of the cause during the compute step.
        """
        pass