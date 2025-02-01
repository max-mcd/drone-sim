# New file: drone-sim/src/models/avoidance_system.py
import numpy as np

class AvoidanceSystem:
    """Manages temporary velocity modifications for collision avoidance"""
    
    def __init__(self):
        self.active = False
        self.timer = 0.0
        self.velocity = None
    
    def apply(self, velocity: np.ndarray, duration: float) -> None:
        """Apply a temporary velocity modification
        
        Args:
            velocity: Modified velocity vector
            duration: How long to maintain this modification (seconds)
        """
        self.active = True
        self.timer = duration
        self.velocity = velocity
    
    def update(self, dt: float) -> None:
        """Update avoidance timer
        
        Args:
            dt: Time step in seconds
        """
        if self.active:
            self.timer -= dt
            if self.timer <= 0:
                self.active = False
                self.velocity = None
    
    @property
    def is_active(self) -> bool:
        return self.active and self.velocity is not None