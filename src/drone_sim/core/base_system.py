class BaseSystem:
    def __init__(self, dependencies=None):
        self.dependencies = dependencies or {}
        self.engine = None
        
    def configure(self, config: dict):
        pass
        
    def update(self, dt: float):
        pass 