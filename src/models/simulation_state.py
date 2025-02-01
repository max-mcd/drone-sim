from dataclasses import dataclass
from typing import Dict, List, Protocol

from .building import Building
from .drone import Drone
from .collision import CollisionRecord


class SimulationEntity(Protocol):
    """Protocol for entities that can be serialized to state"""
    def to_dict(self) -> Dict: ...


@dataclass
class SimulationState:
    """Current state of the simulation.
    
    Provides a consistent snapshot of:
    - Simulation time
    - All entity positions and states
    - Collision records
    """
    time: float
    drones: List[Dict]  # Already serialized
    buildings: List[Dict]  # Keep as Building objects for visualization
    collisions: List[Dict]  # Now serialized

    def __init__(self, time: float, drones: List[Drone], buildings: List[Building],
                 collisions: List[CollisionRecord]):
        """Initialize state, converting all entities to dicts"""
        self.time = time
        self.drones = [drone.to_dict() for drone in drones]
        self.buildings = [building.to_dict() for building in buildings]
        self.collisions = [collision.to_dict() for collision in collisions]