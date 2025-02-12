from abc import ABC, abstractmethod
from typing import Dict

class BaseSystem(ABC):
    """Abstract base class for all simulation systems"""
    
    def __init__(self, dependencies: Dict[str, object]):
        self.dependencies = dependencies
        
    @abstractmethod
    def configure(self, config: dict) -> None:
        """Configure system with parameters from simulation config"""
        pass
        
    @abstractmethod
    def update(self, dt: float) -> None:
        """Update system state by time increment dt"""
        pass 