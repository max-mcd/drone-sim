import logging
from enum import Enum
from typing import List, Tuple

import numpy as np

from drone_sim.models.protocols import Collidable
from drone_sim.models.drone_model import DroneModel
from drone_sim.models.flight_path import FlightPath

logger = logging.getLogger(__name__)

class DroneStatus(Enum):
    ACTIVE = (1, 'active')
    IDLE = (2, 'idle')
    LANDED = (3, 'landed')
    EMERGENCY = (4, 'emergency')
    COLLIDED = (5, 'collided')
    SUCCESSFUL = (6, 'successful')

    def __new__(cls, code: int, label: str):
        obj = object.__new__(cls)
        obj._value_ = code  # Correct way to set enum value
        obj.label = label
        return obj

    def __str__(self):
        return self.label

class Drone(Collidable):
    """Represents an individual drone in the simulation.
    
    This class manages a drone's state and behavior including:
    - Position and velocity tracking
    - Flight path following
    - Collision avoidance
    - Battery monitoring
    - Status updates
    
    The drone follows a flight path while respecting its model's physical limitations
    and avoiding collisions with other drones and buildings.
    
    Attributes:
        id (int): Unique identifier for this drone
        model (DroneModel): Physical specifications and capabilities
        position (np.ndarray): Current 3D position [x,y,z]
        velocity (np.ndarray): Current 3D velocity vector
        status (DroneStatus): Current operational status
        battery_remaining (float): Remaining battery life in seconds
        travel_time (float): Total flight time in seconds
        start_pos (np.ndarray): Starting position
        destination (np.ndarray): Target destination
        flight_path (FlightPath): Path planning system
        successful (bool): Whether mission was completed successfully
        collision_radius (float): Radius of the drone for collision detection
    """
    def __init__(self, drone_id, model: DroneModel, 
                 waypoints: List[Tuple[float, float, float]], priority: int = 0,
                 is_emergency: bool = False):
        super().__init__()
        self.id = drone_id
        self.model = model
        self.waypoints = [np.array(wp, dtype=np.float64) for wp in waypoints]
        self.start_position = self.waypoints[0].copy()
        self._position = self.waypoints[0].copy()
        self.priority = priority
        self.is_emergency = is_emergency
        self.flight_path = FlightPath(self.waypoints)
        self.battery_remaining = model.battery_capacity_s
        self.travel_time = 0.0
        self._velocity = np.zeros(3, dtype=np.float64)  # Use private attribute
        self.status = DroneStatus.ACTIVE
        self.successful = False
        # collision tracking
        self.last_collision_check = 0.0
        self.collision_cooldown = 1.0  # seconds between collision checks
        
        logger.debug("Initialized Drone %d:", self.id)
        logger.debug("  Model: %s (max speed: %.1f m/s)",
                    self.model.name, self.model.max_speed_m_s)
        logger.debug("  Start position: %s", self._position)
        logger.debug("  Waypoints: %s", self.waypoints)

    def move(self, dt: float):
        """Update position based on current velocity"""
        logger.debug("PRE-MOVE | Drone %d @ %s | Velocity: %s | Target: %s",
                   self.id, self.position, self.velocity,
                   self.flight_path.current_waypoint)
        
        # Log pre-move state
        logger.debug("PRE-MOVE | Drone %d:", self.id)
        logger.debug("  Position: %s", self._position)
        logger.debug("  Velocity: %s (speed: %.2f m/s)",
                    self.velocity, np.linalg.norm(self.velocity))
        logger.debug("  Time step: %.3f s", dt)
        
        # Calculate position update
        position_change = self.velocity * dt
        new_position = self._position + position_change
        
        # Update position with altitude limits
        self._position = new_position
        
        # Update timers
        self.battery_remaining -= dt
        self.travel_time += dt
        
        # Log detailed movement
        logger.debug("POST-MOVE | Drone %d:", self.id)
        logger.debug("  Position change: %s (distance: %.2f m)",
                    position_change, np.linalg.norm(position_change))
        logger.debug("  New position: %s", self._position)
        logger.debug("  Battery: %.1f s", self.battery_remaining)
        
        # Check if we've reached current waypoint
        if self.flight_path.is_nearing_waypoint(self.position):
            current_idx = self.flight_path.current_target_index
            if not self.flight_path.advance_waypoint():
                logger.info("Drone %d completed path at %s", self.id, self.position)
                self.successful = True
                self.status = DroneStatus.SUCCESSFUL
            else:
                logger.debug("Drone %d advanced from waypoint %d to %d",
                           self.id, current_idx, self.flight_path.current_target_index)

    @property
    def position(self) -> np.ndarray:
        """Get current position with altitude limits applied"""
        # Ensure z-coordinate is within limits
        if self._position[2] > self.model.max_altitude_m:
            self._position[2] = self.model.max_altitude_m
        elif self._position[2] < 0:
            self._position[2] = 0
        return self._position
    
    @position.setter
    def position(self, value: np.ndarray):
        """Set position with altitude clamping"""
        pos = np.array(value, dtype=np.float64)
        # Only clamp the z-coordinate
        pos[2] = np.clip(pos[2], 0, self.model.max_altitude_m)
        self._position = pos

    @property
    def collision_radius(self) -> float:
        current_speed = np.linalg.norm(self.velocity)
        return self.model.calculate_safety_buffer(current_speed)
        
    def predict_position(self, time_horizon: float) -> np.ndarray:
        return self.position + self.velocity * time_horizon

    def get_safety_buffer(self) -> float:
        """Get minimum safe distance to maintain from other drones"""
        return self.model.calculate_safety_buffer()

    def to_dict(self) -> dict:
        """Convert drone state to a dictionary for serialization
        
        Returns:
            Dictionary containing drone state data
        """
        return {
            'id': self.id,
            'position': self.position.tolist(),
            'velocity': self.velocity.tolist(),
            'status': self.status.value,
            'battery_remaining': self.battery_remaining,
            'flight_path': {
                'waypoints': [wp.tolist() for wp in self.flight_path.waypoints],
                'current_waypoint_index': self.flight_path.current_target_index
            },
            'model': self.model.name,
            'collision_radius': self.collision_radius
        }

    def handle_collision(self):
        self.status = DroneStatus.COLLIDED
        logger.warning(f"Drone {self.id} collision detected!")

    @property
    def velocity(self) -> np.ndarray:
        """Get current velocity vector"""
        return self._velocity
    
    @velocity.setter
    def velocity(self, value: np.ndarray):
        """Set velocity with type conversion"""
        # Convert to float64 numpy array if needed
        if not isinstance(value, np.ndarray):
            value = np.array(value, dtype=np.float64)
        elif value.dtype != np.float64:
            value = value.astype(np.float64)
            
        # Store new velocity
        self._velocity = value
        
        # Log velocity update
        logger.debug("Drone %d velocity updated to %s (speed: %.2f m/s)",
                    self.id, self._velocity, np.linalg.norm(self._velocity))
    
    def update(self, dt: float):
        """Update drone state based on time step"""
        if self.status != DroneStatus.ACTIVE:
            return
        
        # Movement is handled by move() method
        self.move(dt)
