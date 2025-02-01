import logging
from typing import Optional, Tuple

import numpy as np

from ..models.building import Building
from ..models.drone import Drone, DroneStatus

# Configure collision detector logging
logger = logging.getLogger(__name__)

class CollisionDetector:
    """
    A utility class for detecting collisions between drones and other objects in the simulation.
    
    This class provides static methods to check for:
    - Drone-to-drone collisions by comparing their positions and dimensions
    - Drone-to-building collisions by checking if a drone's position falls within a building's bounds
    
    The collision detection uses simplified rectangular bounds checking rather than complex 
    3D geometry to maintain performance while providing reasonable accuracy for the simulation.
    """

    def __init__(self, time_horizon: float = 5.0, prediction_steps: int = 50):
        """Initialize collision detector
        
        Args:
            time_horizon: How far ahead to look for collisions (seconds)
            prediction_steps: Number of points to check along prediction path
        """
        self.time_horizon = time_horizon
        self.prediction_steps = prediction_steps
        logger.info(f"Collision detector initialized: horizon={time_horizon:.1f}s, steps={prediction_steps}")
        
    def check_drone_collision(self, drone1: Drone, drone2: Drone) -> bool:
        """Check for immediate collision between two drones"""
        if drone1.status == DroneStatus.COLLIDED or drone2.status == DroneStatus.COLLIDED:
            return False

        # Use smaller buffer if either drone is already yielding
        is_yielding = drone1._avoidance.is_active or drone2._avoidance.is_active
        
        if is_yielding:
            # Use physical dimensions plus small safety margin for actual collision
            collision_buffer = max(
                max(drone1.model.dimensions['length'], drone1.model.dimensions['width']) * 1.5 + 10.0,
                max(drone2.model.dimensions['length'], drone2.model.dimensions['width']) * 1.5 + 10.0
            )
        else:
            # Use intermediate buffer for non-yielding drones
            collision_buffer = max(
                max(drone1.model.dimensions['length'], drone1.model.dimensions['width']) * 2.0 + 15.0,
                max(drone2.model.dimensions['length'], drone2.model.dimensions['width']) * 2.0 + 15.0
            )

        pos1, pos2 = drone1.position, drone2.position
        distance = np.linalg.norm(pos1 - pos2)
        vertical_sep = abs(pos1[2] - pos2[2])

        if distance < collision_buffer and vertical_sep < 30.0:
            logger.warning(f"""
                Immediate collision detected:
                Distance: {distance:.1f}m
                Vertical separation: {vertical_sep:.1f}m
                Collision buffer: {collision_buffer:.1f}m
                Yielding active: {is_yielding}
                Drone {drone1.id} position: {pos1}
                Drone {drone2.id} position: {pos2}
            """)
            return True
        
        return False

    @staticmethod
    def check_building_collision(drone: Drone, building: Building) -> bool:
        """Check for collision between drone and building"""
        # Skip if drone has already collided
        if drone.status == DroneStatus.COLLIDED:
            return False
            
        drone_pos = drone.position
        
        # Get half dimensions of drone for expanding building bounds
        half_length = drone.model.dimensions['length'] / 2
        half_width = drone.model.dimensions['width'] / 2
        half_height = drone.model.dimensions['height'] / 2
        
        # Check if drone intersects with expanded building bounds
        in_x_bounds = (building.x - building.width/2 - half_width <= drone_pos[0] <= 
                      building.x + building.width/2 + half_width)
        in_y_bounds = (building.y - building.length/2 - half_length <= drone_pos[1] <= 
                      building.y + building.length/2 + half_length)
        in_z_bounds = (0 - half_height <= drone_pos[2] <= 
                      building.height + half_height)
        
        return in_x_bounds and in_y_bounds and in_z_bounds

    def check_future_collision(self, drone1: Drone, drone2: Drone) -> Optional[float]:
        """Check if two drones will collide within the time horizon"""
        if drone1.status != DroneStatus.ACTIVE or drone2.status != DroneStatus.ACTIVE:
            return None
        
        # Always use full safety buffer for future collision prediction
        # This is used by collision avoidance system
        safety_distance = max(
            drone1.get_safety_buffer(),
            drone2.get_safety_buffer()
        )
        
        # Check vertical separation first
        vertical_separation = abs(drone1.position[2] - drone2.position[2])
        if vertical_separation > 30.0:  # Significant vertical separation
            logger.debug(f"Sufficient vertical separation: {vertical_separation:.1f}m")
            return None

        # Log initial check
        current_distance = np.linalg.norm(drone1.position - drone2.position)
        logger.debug(f"""
            Starting future collision check:
            Current distance: {current_distance:.1f}m
            Vertical separation: {vertical_separation:.1f}m
            Required separation: {safety_distance:.1f}m
            Time horizon: {self.time_horizon:.1f}s
            Check points: {self.prediction_steps}
        """)

        # Check positions at multiple future times
        time_steps = np.linspace(0, self.time_horizon, self.prediction_steps)
        
        for t in time_steps:
            pos1 = drone1.predict_position(t)
            pos2 = drone2.predict_position(t)
            
            distance = np.linalg.norm(pos1 - pos2)
            vertical_sep = abs(pos1[2] - pos2[2])
            
            if distance < safety_distance and vertical_sep < 30.0:
                logger.info(f"""
                    Future collision predicted:
                    Time: {t:.1f}s ahead
                    Distance: {distance:.1f}m
                    Vertical separation: {vertical_sep:.1f}m
                    Required separation: {safety_distance:.1f}m
                    Drone {drone1.id} position: {pos1}
                    Drone {drone2.id} position: {pos2}
                    Relative velocity: {np.linalg.norm(drone1.velocity - drone2.velocity):.1f} m/s
                """)
                return t
                
        return None
        
    def calculate_closest_approach(
        self, 
        drone1: Drone,
        drone2: Drone
    ) -> Tuple[float, float]:
        """Calculate time and distance of closest approach
        
        Returns:
            Tuple of (time to closest approach, minimum distance)
        """
        pos1 = drone1.position
        pos2 = drone2.position
        vel1 = drone1.velocity
        vel2 = drone2.velocity
        
        # Calculate relative motion
        rel_pos = pos1 - pos2
        rel_vel = vel1 - vel2
        
        # If relative velocity is zero, return current distance
        if np.allclose(rel_vel, 0):
            return 0.0, np.linalg.norm(rel_pos)
            
        # Time of closest approach = -dot(rel_pos, rel_vel) / dot(rel_vel, rel_vel)
        time = -np.dot(rel_pos, rel_vel) / np.dot(rel_vel, rel_vel)
        
        # Only consider future times within horizon
        time = min(max(0, time), self.time_horizon)
        
        # Calculate positions at closest approach
        future_pos1 = drone1.predict_position(time)
        future_pos2 = drone2.predict_position(time)
        min_distance = np.linalg.norm(future_pos1 - future_pos2)
        
        return time, min_distance