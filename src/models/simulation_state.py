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

    def __init__(self, time: float, drones: List[dict], buildings: List[dict],
                 drone_collisions: List[Tuple], building_collisions: List[Tuple]):
        self.time = time
        self.drones = [{
            'id': d['id'],
            'position': d['position'],
            'velocity': d['velocity'],
            'start_pos': d['start_pos'],
            'destination': d['destination'],
            'successful': d['successful'],
            'status': d['status']  # Add status to state
        } for d in drones]
        self.buildings = buildings
        self.drone_collisions = drone_collisions
        self.building_collisions = building_collisions 