# New file: drone-sim/src/models/collision_record.py
from dataclasses import dataclass
from typing import Optional, Tuple

@dataclass
class CollisionRecord:
    """Record of a collision event in the simulation
    
    Attributes:
        drone_id: ID of the primary drone involved
        other_id: ID of other entity (drone or building)
        timestamp: Time of collision in seconds
        position: Optional (x,y,z) position for building collisions
        collision_type: Either 'drone' or 'building'
    """
    drone_id: int
    other_id: int  # Drone or building ID
    timestamp: float
    position: Optional[Tuple[float, float, float]] = None  # Optional for drone-drone collisions
    collision_type: str = 'drone'  # 'drone' or 'building'

    @classmethod
    def from_drone_collision(cls, drone1_id: int, drone2_id: int, time: float) -> 'CollisionRecord':
        """Create a record for a drone-drone collision"""
        return cls(
            drone_id=drone1_id,
            other_id=drone2_id,
            timestamp=time,
            collision_type='drone'
        )

    @classmethod
    def from_building_collision(cls, drone_id: int, building_id: int, time: float, 
                              pos: Tuple[float, float, float]) -> 'CollisionRecord':
        """Create a record for a drone-building collision"""
        return cls(
            drone_id=drone_id,
            other_id=building_id,
            timestamp=time,
            position=pos,
            collision_type='building'
        )