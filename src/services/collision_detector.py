import logging

import numpy as np

from ..models.building import Building
from ..models.drone import Drone

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

    COLLISION_THRESHOLD = 10.0  # meters
    @staticmethod
    def check_drone_collision(drone1: Drone, drone2: Drone) -> bool:
        """Check for collision between two drones"""
        # Skip if either drone has already collided
        if drone1.status == 'collided' or drone2.status == 'collided':
            return False

        pos1 = drone1.position
        pos2 = drone2.position
        distance = np.linalg.norm(pos1 - pos2)
        
        # Debug log when drones are getting close
        if distance < 50.0:
            logger.info("Checking potential collision...")

        return distance < CollisionDetector.COLLISION_THRESHOLD

    @staticmethod
    def check_building_collision(drone: Drone, building: Building) -> bool:
        """Check for collision between drone and building"""
        # Skip if drone has already collided
        if drone.status == 'collided':
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