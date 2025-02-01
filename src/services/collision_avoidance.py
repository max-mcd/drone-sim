import logging

import numpy as np

from ..models.drone import Drone
from .collision_detector import CollisionDetector

logger = logging.getLogger(__name__)

class CollisionAvoidanceSystem:
    """Manages collision avoidance between drones in the simulation.
    
    This system:
    - Detects potential collisions using the CollisionDetector
    - Determines which drone should yield in potential collision scenarios
    - Calculates safe speeds and trajectories for collision avoidance
    - Applies avoidance maneuvers while maintaining safety buffers
    
    The avoidance logic uses a hierarchical decision system considering:
    - Emergency status (future capability) 
    - Vertical separation between drones
    - Progress towards destination
    - Drone ID comparison as final tiebreaker
    """
    def __init__(self, time_horizon: float = 5.0):
        """Initialize collision avoidance system"""
        self.detector = CollisionDetector(time_horizon=time_horizon)
        logger.info("Collision avoidance system initialized with time horizon: %.1fs", time_horizon)
        
    def _should_yield(self, drone1: Drone, drone2: Drone) -> bool:
        """Hierarchical decision making for drone yielding
        
        Priority order:
        1. Emergency status (future)
        2. Vertical separation
        3. Progress to destination
        4. ID comparison
        """
        # Check vertical separation first
        altitude_diff = abs(drone1.position[2] - drone2.position[2])
        if altitude_diff > 10:  # Different altitude layers
            logger.debug(f"Drones at different altitudes ({altitude_diff:.1f}m), no yield needed")
            return False
            
        # Calculate progress ratio for each drone
        def get_progress(drone: Drone) -> float:
            total_distance = np.linalg.norm(drone.destination - drone.start_pos)
            current_progress = np.linalg.norm(drone.position - drone.start_pos)
            return current_progress / total_distance
            
        progress1 = get_progress(drone1)
        progress2 = get_progress(drone2)
        
        logger.debug(f"""
            Yield decision factors:
            Drone {drone1.id}: progress={progress1:.2f}, altitude={drone1.position[2]:.1f}m
            Drone {drone2.id}: progress={progress2:.2f}, altitude={drone2.position[2]:.1f}m
        """)
        
        # If progress difference is significant
        if abs(progress1 - progress2) > 0.1:
            logger.debug(f"Yield decision based on progress: Drone {drone1.id if progress1 > progress2 else drone2.id} should yield")
            return progress1 > progress2
            
        logger.debug(f"Yield decision based on ID: Drone {drone1.id if drone1.id > drone2.id else drone2.id} should yield")
        return drone1.id > drone2.id
        
    def _calculate_safe_speed(
        self,
        yielding_drone: Drone,
        passing_drone: Drone,
        required_separation: float
    ) -> float:
        """Calculate safe speed for yielding drone to maintain safe separation.
        
        Uses time and distance of closest approach to determine appropriate speed reduction.
        Applies progressively more aggressive slowdown as separation distance decreases:
        - Normal slowdown when within required separation
        - Cubic slowdown within 75% of required separation  
        - Emergency brake (50% additional reduction) within 60% of required separation
        
        Args:
            yielding_drone: The drone that should yield
            passing_drone: The drone being yielded to
            required_separation: Minimum required separation distance in meters
            
        Returns:
            float: Safe speed in m/s for yielding drone
        """
        # Get time and distance of closest approach
        time_to_closest, min_distance = self.detector.calculate_closest_approach(
            yielding_drone, passing_drone
        )
        
        logger.debug(f"""
            Safe speed calculation:
            Time to closest: {time_to_closest:.1f}s
            Min distance: {min_distance:.1f}m
            Required separation: {required_separation:.1f}m
        """)
        
        if time_to_closest <= 0:
            return yielding_drone.model.max_speed
            
        # Calculate required speed reduction
        if min_distance < required_separation:
            # Reduce speed proportionally to separation violation
            violation_ratio = min_distance / required_separation
            # Much more aggressive slowdown for close encounters
            if min_distance < required_separation * 0.75:  # Within 75% of required separation
                speed_factor = max(0.05, (min_distance / required_separation) ** 3)
            else:
                speed_factor = max(0.1, violation_ratio * violation_ratio)
            
            current_speed = np.linalg.norm(yielding_drone.velocity)
            new_speed = current_speed * speed_factor
            
            # Add emergency brake if getting too close
            if min_distance < required_separation * 0.6:  # Within 60% of required separation
                new_speed *= 0.5  # Additional 50% speed reduction
            
            logger.debug(f"""
                Speed adjustment:
                Current speed: {current_speed:.1f} m/s
                Speed factor: {speed_factor:.2f}
                Emergency brake: {min_distance < required_separation * 0.6}
                New speed: {new_speed:.1f} m/s
            """)
            
            return new_speed
            
        return yielding_drone.model.max_speed
        
    def resolve_conflict(self, drone1: Drone, drone2: Drone) -> None:
        """Resolve potential collision between two drones by:
        1. Checking if collision avoidance is needed based on future trajectory
        2. Determining which drone should yield based on priority rules
        3. Calculating required separation distance between drones
        4. Computing and applying speed adjustments to the yielding drone
        
        Args:
            drone1: First drone to check for collision
            drone2: Second drone to check for collision
        """
        # Check if avoidance is needed
        collision_time = self.detector.check_future_collision(drone1, drone2)
        if collision_time is None:
            return
            
        logger.info(f"""
            Potential collision detected:
            Time to collision: {collision_time:.1f}s
            Drone {drone1.id} position: {drone1.position}
            Drone {drone2.id} position: {drone2.position}
            Distance: {np.linalg.norm(drone1.position - drone2.position):.1f}m
        """)
        
        # Determine which drone should yield
        if self._should_yield(drone1, drone2):
            yielding_drone = drone1
            passing_drone = drone2
        else:
            yielding_drone = drone2
            passing_drone = drone1
            
        # Calculate required separation
        required_separation = max(
            yielding_drone.get_safety_buffer(),
            passing_drone.get_safety_buffer()
        )
        
        # Calculate and apply speed adjustment
        new_speed = self._calculate_safe_speed(
            yielding_drone,
            passing_drone,
            required_separation
        )
        
        # Apply speed adjustment by scaling velocity
        current_speed = np.linalg.norm(yielding_drone.velocity)
        speed_ratio = 1.0  # Default no change
        if current_speed > 0:
            speed_ratio = new_speed / current_speed
            new_velocity = yielding_drone.velocity * speed_ratio
            # Apply modification with 2-second duration
            yielding_drone.apply_avoidance_velocity(new_velocity, duration=2.0)
            
        logger.info(f"""
            Collision avoidance applied:
            Yielding drone: {yielding_drone.id}
            Original speed: {current_speed:.1f} m/s
            New speed: {new_speed:.1f} m/s
            Required separation: {required_separation:.1f}m
            Speed ratio: {speed_ratio:.2f}
        """) 