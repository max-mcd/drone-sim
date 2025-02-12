from typing import List
from .base import AvoidanceStrategy
from drone_sim.models.collision import CollisionRecord

class NoOpAvoidance(AvoidanceStrategy):
    """Default strategy that does no avoidance"""
    
    def resolve(self, collisions: List[CollisionRecord]) -> List[CollisionRecord]:
        return collisions  # No action taken 