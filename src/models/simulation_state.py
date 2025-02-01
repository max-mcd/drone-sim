from dataclasses import dataclass
from typing import Dict, List, Tuple, Union

from .building import Building
from .drone import Drone
from .collision import CollisionRecord


@dataclass
class SimulationState:
    """Represents the complete state of the simulation at a point in time.
    
    Attributes:
        time: Current simulation time in seconds
        drones: List of drone states containing position, velocity, waypoints, etc.
        buildings: List of building objects in the simulation
        collisions: List of collision records
    """
    time: float
    drones: List[Dict[str, Union[int, List[float], bool, str]]]
    buildings: List[Building]
    collisions: List[CollisionRecord]

    def __init__(self, time: float, drones: List[Drone], buildings: List[Building],
                 collisions: List[CollisionRecord]):
        """Initialize simulation state.
        
        Converts Drone objects to dictionary format for visualization while keeping
        Building objects intact for direct property access.
        """
        self.time = time
        self.drones = [drone.to_dict() for drone in drones]
        self.buildings = buildings
        self.collisions = collisions