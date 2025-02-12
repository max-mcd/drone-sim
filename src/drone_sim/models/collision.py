# drone-sim/src/models/collision.py
from dataclasses import dataclass
from typing import Dict, Optional, Tuple
from .protocols import SerializableEntity


@dataclass
class CollisionRecord:
    """Represents a collision event between two entities"""
    
    participants: tuple  # IDs of involved entities
    timestamp: float     # Simulation time of collision
    collision_type: str  # 'drone' or 'building'
    avoided: bool = False  # Whether collision was prevented
    position: tuple = None  # (x,y,z) collision coordinates

    @classmethod
    def from_drone_collision(cls, drone1_id: int, drone2_id: int, 
                           timestamp: float, position: tuple = None):
        """Create drone-drone collision record"""
        return cls(
            participants=(drone1_id, drone2_id),
            timestamp=timestamp,
            collision_type='drone',
            position=position
        )

    @classmethod
    def from_building_collision(cls, drone_id: int, building_id: int, 
                              timestamp: float, position: tuple):
        """Create drone-building collision record"""
        return cls(
            participants=(drone_id, building_id),
            timestamp=timestamp,
            collision_type='building',
            position=position
        )

    def to_dict(self):
        """Serialize for event bus/state management"""
        return {
            'participants': self.participants,
            'timestamp': self.timestamp,
            'type': self.collision_type,
            'avoided': self.avoided,
            'position': self.position
        }

    @property
    def building_id(self) -> int:
        """For building collisions, other_id is the building ID"""
        if self.collision_type != 'building':
            raise AttributeError("Not a building collision")
        return self.participants[1]

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

    @property
    def drone_id(self) -> int:
        """Get drone ID for both collision types"""
        if self.collision_type == 'drone':
            return self.participants[0]
        return self.participants[0]  # For building collisions, first participant is always drone

    @property
    def other_id(self) -> int:
        """For drone-drone collisions, the second participant is the other ID"""
        if self.collision_type != 'drone':
            raise AttributeError("Not a drone-drone collision")
        return self.participants[1]