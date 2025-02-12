import time


class SimulationError(Exception):
    """Base simulation exception"""
    def __init__(self, message: str, system: str = None):
        super().__init__(message)
        self.system = system
        self.timestamp = time.time()

class CollisionError(SimulationError):
    """Error in collision detection/resolution"""
    def __init__(self, message: str):
        super().__init__(message, system='collision')

class PhysicsError(SimulationError):
    """Error in physics calculations"""
    def __init__(self, message: str):
        super().__init__(message, system='physics')

class PathfindingError(SimulationError):
    def __init__(self, message: str):
        super().__init__(message, system='pathfinding') 