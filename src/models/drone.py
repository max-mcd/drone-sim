import logging

import numpy as np

from .drone_model import DroneModel  # Import DroneModel from correct file
from .flight_path import FlightPath  # Import FlightPath from correct file

logger = logging.getLogger(__name__)


class Drone:
    def __init__(self, id: int, model: DroneModel, flight_path: FlightPath):
        self.id = id
        self.model = model
        self.flight_path = flight_path
        self.position = flight_path.waypoints[0].copy()
        self.start_pos = flight_path.waypoints[0].copy()
        self.destination = flight_path.waypoints[-1].copy()
        self.velocity = np.zeros(3)
        self.battery_remaining = model.battery_life * 3600  # Convert to seconds
        self.successful = False
        self.travel_time = 0.0
        self.status = 'active'  # Can be: 'active', 'collided', 'successful'

    def update(self, dt: float) -> bool:
        """Update drone position and state"""
        if self.status != 'active':
            logger.info(f"Drone {self.id} inactive...")
            return False
            
        current_waypoint = self.flight_path.get_next_waypoint()
        to_waypoint = current_waypoint - self.position
        distance = np.linalg.norm(to_waypoint)
        
        logger.info(f"""
            Drone {self.id} update:
            Distance to waypoint: {distance:.1f}m
            Current position: {self.position}
            Waypoint: {current_waypoint}
            Status: {self.status}
        """)
        
        if distance < 1.0:  # Reached waypoint
            if not self.flight_path.advance_waypoint():
                self.successful = True
                self.status = 'successful'
                self.velocity = np.zeros(3)  # Stop moving
                self.position = current_waypoint.copy()  # Snap to final position
                return True
            return True  # Skip movement this frame after advancing waypoint
        
        # Only calculate direction if we have distance to travel
        direction = to_waypoint / distance
        
        # Reduce speed when close to waypoint
        if distance < 10.0:  # Within 10m
            speed_factor = max(0.1, distance / 10.0)  # Slow down gradually
            self.velocity = direction * self.model.max_speed * speed_factor
            logger.debug(f"""
                Speed reduction active:
                Distance: {distance:.1f}m
                Speed factor: {speed_factor:.2f}
                Reduced speed: {np.linalg.norm(self.velocity):.1f} m/s
            """)
        else:
            self.velocity = direction * self.model.max_speed
        
        # Update position
        self.position += self.velocity * dt
        self.travel_time += dt
        
        # Debug logging
        logger.debug(f"""
            Drone {self.id} update:
            Model: {self.__class__.__name__}
            Max speed from model: {self.model.max_speed} m/s
            Raw direction vector: {to_waypoint}
            Normalized direction: {direction}
            Calculated velocity: {self.velocity}
            Actual speed: {np.linalg.norm(self.velocity)} m/s
            Position: {self.position}
            Waypoint: {current_waypoint}
        """)
        
        return True
