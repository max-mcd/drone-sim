from dataclasses import dataclass
from typing import Dict
import numpy as np

@dataclass
class DroneModel:
    max_speed: float
    range: float
    dimensions: Dict[str, float]
    payload_capacity: float
    battery_life: float

class Drone:
    def __init__(self, id: int, model: DroneModel, start_pos: np.ndarray, destination: np.ndarray):
        self.id = id
        self.model = model
        self.position = start_pos
        self.destination = destination
        self.velocity = np.zeros(3)
        self.battery_remaining = model.battery_life * 3600  # Convert to seconds

    def update(self, dt: float) -> bool:
        """
        Updates the drone's position and battery status for the given time step.
        
        Args:
            dt (float): Time step in seconds
            
        Returns:
            bool: True if drone is still operational, False if battery depleted
        """
        if self.battery_remaining <= 0:
            return False

        # This block updates the drone's position and velocity:
        # 1. Calculates direction vector to destination
        # 2. Gets distance to destination
        # 3. Normalizes direction vector if distance > 0
        # 4. Sets velocity based on max speed or required speed to reach destination
        # 5. Updates position based on velocity and time step
        # 6. Decrements remaining battery time

        direction = self.destination - self.position
        distance = np.linalg.norm(direction)
        
        if distance > 0:
            direction = direction / distance # Normalize direction vector   
            speed = min(self.model.max_speed, distance / dt) 
            self.velocity = direction * speed
            self.position += self.velocity * dt
            self.battery_remaining -= dt
                
        return True
