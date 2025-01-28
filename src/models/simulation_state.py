from dataclasses import dataclass
from typing import Dict, List, Tuple


@dataclass
class SimulationState:
    """Represents the complete state of the simulation at a point in time"""
    time: float
    drones: List[Dict[str, List[float]]]  # position, velocity
    buildings: List[Dict[str, List[float]]]  # position, dimensions
    drone_collisions: List[Tuple[int, int, float]]  # drone1_id, drone2_id, time
    building_collisions: List[Tuple[int, int, float, float, float, float]]  # drone_id, building_id, time, x, y, z 