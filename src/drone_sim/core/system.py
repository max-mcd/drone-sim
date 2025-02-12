from drone_sim.models.protocols import Configurable, Updatable


class BaseSystem(Updatable, Configurable):
    def __init__(self, dependencies: dict):
        self.dependencies = dependencies
        
    def configure(self, config: dict):
        """Default configuration handler"""
        pass
        
    def update(self, dt: float):
        """Default update handler"""
        pass 