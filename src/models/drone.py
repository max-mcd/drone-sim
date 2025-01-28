import logging

import numpy as np

from .drone_model import DroneModel  # Import DroneModel from correct file

logger = logging.getLogger(__name__)


class Drone:
    def __init__(self, id: int, model: DroneModel, start_pos: np.ndarray, destination: np.ndarray):
        self.id = id
        self.model = model
        self.position = start_pos.copy()  # Current position
        self.start_pos = start_pos.copy()  # Store initial position
        self.destination = destination
        self.velocity = np.zeros(3)
        self.battery_remaining = model.battery_life * 3600  # Convert to seconds
        self.successful = False
        self.travel_time = 0.0
        self.logger = logging.getLogger(__name__)
        # Set logging level to INFO or higher to suppress debug messages
        self.logger.setLevel(logging.INFO) # TODO: Change to DEBUG for more detailed logging

    def update(self, dt: float) -> bool:
        """Update drone position and state"""
        if self.successful:
            return False
            
        # Calculate vector to destination
        to_destination = self.destination - self.position
        distance = np.linalg.norm(to_destination)
        
        if distance < 1.0:  # Within 1m counts as arrived
            self.successful = True
            self.velocity = np.zeros(3)
            return False
            
        # Update velocity (normalized direction * max_speed)
        direction = to_destination / distance
        self.velocity = direction * self.model.max_speed
        
        # Update position
        self.position += self.velocity * dt
        self.travel_time += dt
        
        # Debug logging
        logger.debug(f"""
            Drone {self.id} update:
            Model: {self.__class__.__name__}
            Max speed from model: {self.model.max_speed} m/s
            Raw direction vector: {to_destination}
            Normalized direction: {direction}
            Calculated velocity: {self.velocity}
            Actual speed: {np.linalg.norm(self.velocity)} m/s
            Position: {self.position}
            Destination: {self.destination}
        """)
        
        return True
