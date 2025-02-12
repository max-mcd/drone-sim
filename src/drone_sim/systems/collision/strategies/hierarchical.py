import logging
from typing import List

import numpy as np

from drone_sim.models.collision import CollisionRecord
from drone_sim.models.drone import Drone

from .base import AvoidanceStrategy

logger = logging.getLogger(__name__)

class HierarchicalAvoidance(AvoidanceStrategy):
    def __init__(self, priority_mode: str, min_speed_factor: float, max_speed_factor: float, replan_distance: float):
        self.priority_mode = priority_mode
        self.min_speed_factor = min_speed_factor
        self.max_speed_factor = max_speed_factor
        self.replan_distance = replan_distance

    def resolve(self, collisions: List[CollisionRecord]) -> List[CollisionRecord]:
        for collision in collisions:
            # Restore vector-based avoidance
            avoidance_vector = collision.drone1.position - collision.drone2.position
            if np.linalg.norm(avoidance_vector) > 0:
                avoidance_dir = avoidance_vector / np.linalg.norm(avoidance_vector)
                # Use configured speed factors
                max_speed = min(collision.drones, key=lambda d: np.linalg.norm(d.velocity)).model.max_speed
                new_velocity = avoidance_dir * max_speed * self.max_speed_factor
                collision.drone1.velocity = new_velocity
            logger.info(f"Resolved collision between {collision.drone1.id}")
            
        return collisions

    def _should_yield(self, drone1: Drone, drone2: Drone) -> bool:
        # Altitude priority: lower altitude drones yield
        if drone1.position[2] < drone2.position[2]:
            return True
        elif drone1.position[2] > drone2.position[2]:
            return False
        if drone1.priority < drone2.priority:
            return True
        elif drone1.progress > drone2.progress:
            return False
        return drone1.id < drone2.id

    def _calculate_avoidance_velocity(self, drone: Drone, other: Drone) -> np.ndarray:
        avoidance_vector = drone.position - other.position
        if np.linalg.norm(avoidance_vector) > 0:
            avoidance_direction = avoidance_vector / np.linalg.norm(avoidance_vector)
            # Dynamic reduction based on collision imminence
            time_to_collision = self.detector.calculate_closest_approach(drone, other)[0]
            reduction = max(0.3, min(0.7, time_to_collision / self.detector.time_horizon))
            return avoidance_direction * drone.model.max_speed * reduction
        return drone.velocity 