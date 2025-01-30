import logging

import numpy as np
from typing import List, Optional, Tuple

from .drone_model import DroneModel  # Import DroneModel from correct file
from .flight_path import FlightPath  # Import FlightPath from correct file

logger = logging.getLogger(__name__)


class Drone:
    def __init__(self, id: int, model: DroneModel, start: np.ndarray, 
                 destination: np.ndarray, flight_path: FlightPath):
        self.id = id
        self.model = model
        self.position = start.copy()
        self._base_velocity = np.zeros(3)
        self._avoidance_velocity = None
        self._avoidance_timer = 0.0
        self.flight_path = flight_path
        self.status = 'active'  # 'active', 'successful', or 'collided'
        self.battery_remaining = model.battery_life * 3600  # Convert to seconds
        self.successful = False
        self.travel_time = 0.0
        self.start_pos = start.copy()
        self.destination = destination.copy()

    @property
    def velocity(self) -> np.ndarray:
        """Get current velocity, considering avoidance adjustments"""
        if self._avoidance_velocity is not None:
            return self._avoidance_velocity
        return self._base_velocity

    @velocity.setter
    def velocity(self, new_velocity: np.ndarray):
        """Set base velocity"""
        self._base_velocity = new_velocity

    def apply_avoidance_velocity(self, new_velocity: np.ndarray, duration: float = 2.0):
        """Apply temporary velocity modification for collision avoidance
        
        Args:
            new_velocity: Modified velocity vector
            duration: How long to maintain this modification (seconds)
        """
        self._avoidance_velocity = new_velocity
        self._avoidance_timer = duration
        logger.debug(f"""
            Applied avoidance velocity to Drone {self.id}:
            Original: {self._base_velocity}
            Modified: {new_velocity}
            Duration: {duration:.1f}s
        """)

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
        
        # Update avoidance timer
        if self._avoidance_timer > 0:
            self._avoidance_timer -= dt
            if self._avoidance_timer <= 0:
                self._avoidance_velocity = None
                logger.debug(f"Drone {self.id} returning to normal velocity")
        
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

    def predict_position(self, time_horizon: float) -> np.ndarray:
        """Predict drone position after time_horizon seconds
        
        Args:
            time_horizon: Time in seconds to predict ahead
            
        Returns:
            Predicted position as numpy array [x, y, z]
        """
        if self.status != 'active':
            return self.position
        
        # Get current waypoint and next waypoint
        current_wp = self.flight_path.current_waypoint
        next_wp = self.flight_path.next_waypoint
        
        if next_wp is None:
            # At final waypoint, assume hover
            return self.position
        
        # Calculate direction of travel
        direction = next_wp - current_wp
        distance_to_next = np.linalg.norm(direction)
        if distance_to_next > 0:
            direction = direction / distance_to_next
        
        # Predict considering current velocity and waypoint path
        predicted_pos = self.position + self.velocity * time_horizon
        
        # Constrain to flight path
        if np.linalg.norm(predicted_pos - current_wp) > distance_to_next:
            # Would overshoot next waypoint
            predicted_pos = next_wp
        
        return predicted_pos

    def get_safety_buffer(self) -> float:
        """Get minimum safe distance to maintain from other drones"""
        return self.model.calculate_safety_buffer()
