import numpy as np

from ..models.building import Building
from ..models.drone import Drone


class CollisionDetector:
    """
    A utility class for detecting collisions between drones and other objects in the simulation.
    
    This class provides static methods to check for:
    - Drone-to-drone collisions by comparing their positions and dimensions
    - Drone-to-building collisions by checking if a drone's position falls within a building's bounds
    
    The collision detection uses simplified rectangular bounds checking rather than complex 
    3D geometry to maintain performance while providing reasonable accuracy for the simulation.
    """
    @staticmethod
    def check_drone_collision(drone1: Drone, drone2: Drone) -> bool:
        """
        Checks if two drones have collided by comparing their positions and dimensions.
        
        Uses a simplified collision detection approach where each drone is treated as a sphere
        with radius equal to half of its largest dimension. A collision occurs if the distance 
        between drone centers is less than the sum of their radii.
        
        Args:
            drone1: First drone to check for collision
            drone2: Second drone to check for collision
            
        Returns:
            bool: True if drones have collided (are closer than min safe distance), False otherwise
        """
        # Calculate radius for each drone (half of largest dimension)
        radius1 = max(
            drone1.model.dimensions['length'],
            drone1.model.dimensions['width']
        ) / 2
        
        radius2 = max(
            drone2.model.dimensions['length'],
            drone2.model.dimensions['width']
        ) / 2
        
        # Minimum safe distance is sum of both radii
        min_distance = radius1 + radius2
        
        return np.linalg.norm(drone1.position - drone2.position) < min_distance

    @staticmethod
    def check_building_collision(drone: Drone, building: Building) -> bool:
        """
        Check if a drone has collided with a building by testing if any part of the drone
        intersects with the building's 3D bounds. The collision bounds are expanded by
        half of the drone's dimensions in each direction.

        Args:
            drone: The drone object to check for collision
            building: The building object to check for collision with

        Returns:
            bool: True if the drone intersects with the building bounds, False otherwise
        """
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