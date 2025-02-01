# drone-sim/src/models/collision.py
from dataclasses import dataclass
from typing import Optional, Tuple, Dict

@dataclass
class CollisionRecord:
    """Record of a collision event in the simulation.
    
    Stores details about collisions between drones or between drones and buildings,
    including the IDs of objects involved, timestamp, position coordinates, and type.
    
    Attributes:
        drone_id: ID of the primary drone involved in collision
        other_id: ID of other drone or building involved
        timestamp: Time in seconds when collision occurred
        position: Optional (x,y,z) coordinates of collision point
        collision_type: Either 'drone' for drone-drone or 'building' for drone-building
    """
    
    drone_id: int
    other_id: int  # Drone or building ID
    timestamp: float
    position: Optional[Tuple[float, float, float]] = None
    collision_type: str = 'drone'  # 'drone' or 'building'

    @property
    def building_id(self) -> int:
        """For building collisions, other_id is the building ID"""
        if self.collision_type != 'building':
            raise AttributeError("Not a building collision")
        return self.other_id

    @property
    def time(self) -> float:
        """Alias for timestamp to maintain compatibility"""
        return self.timestamp

    @property
    def x(self) -> float:
        """X coordinate of collision position"""
        if not self.position:
            raise AttributeError("No position data")
        return self.position[0]

    @property
    def y(self) -> float:
        """Y coordinate of collision position"""
        if not self.position:
            raise AttributeError("No position data")
        return self.position[1]

    @property
    def z(self) -> float:
        """Z coordinate of collision position"""
        if not self.position:
            raise AttributeError("No position data")
        return self.position[2]

    @classmethod
    def from_drone_collision(cls, drone1_id: int, drone2_id: int, timestamp: float) -> 'CollisionRecord':
        """Create record for drone-drone collision"""
        return cls(
            drone_id=drone1_id,
            other_id=drone2_id,
            timestamp=timestamp,
            collision_type='drone'
        )

    @classmethod
    def from_building_collision(cls, drone_id: int, building_id: int, 
                              timestamp: float, position: Tuple[float, float, float]) -> 'CollisionRecord':
        """Create record for drone-building collision"""
        return cls(
            drone_id=drone_id,
            other_id=building_id,
            timestamp=timestamp,
            position=position,
            collision_type='building'
        )

    def to_dict(self) -> Dict:
        """Convert collision record to dictionary"""
        data = {
            'drone_id': self.drone_id,
            'other_id': self.other_id,
            'timestamp': self.timestamp,
            'collision_type': self.collision_type
        }
        if self.position:
            data['position'] = self.position
        return data