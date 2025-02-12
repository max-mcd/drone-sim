from typing import List

from drone_sim.models.collision import CollisionRecord
from drone_sim.models.drone import DroneStatus

from .base import AbstractStrategy


class EmergencyAvoidance(AbstractStrategy):
    """Extends hierarchical with emergency vehicle priority"""
    
    def resolve(self, collisions: List[CollisionRecord]) -> List[CollisionRecord]:
        # First handle emergency collisions
        emergencies = [c for c in collisions if c.drone1.is_emergency or c.drone2.is_emergency]
        non_emergencies = [c for c in collisions if c not in emergencies]
        
        # Resolve emergencies with priority
        for collision in emergencies:
            if collision.drone1.is_emergency:
                collision.drone2.status = DroneStatus.YIELDING
            else:
                collision.drone1.status = DroneStatus.YIELDING
        
        return super().resolve(non_emergencies) + emergencies

    def _is_emergency(self, collision: CollisionRecord) -> bool:
        return collision.drone1.is_emergency or collision.drone2.is_emergency 