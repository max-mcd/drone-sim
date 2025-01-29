from dataclasses import dataclass
from typing import Dict, List, Tuple, Union

from .building import Building
from .drone import Drone


@dataclass
class SimulationState:
    """Represents the complete state of the simulation at a point in time.
    
    Attributes:
        time: Current simulation time in seconds
        drones: List of drone states containing position, velocity, waypoints, etc.
        buildings: List of building objects in the simulation
        drone_collisions: List of drone-drone collisions (id1, id2, time)
        building_collisions: List of drone-building collisions (drone_id, building_id, time, x, y, z)
    """
    time: float
    drones: List[Dict[str, Union[int, List[float], bool, str]]]
    buildings: List[Building]
    drone_collisions: List[Tuple[int, int, float]]
    building_collisions: List[Tuple[int, int, float, float, float, float]]

    def __init__(self, time: float, drones: List[Drone], buildings: List[Building],
                 drone_collisions: List[Tuple], building_collisions: List[Tuple]):
        """Initialize simulation state.
        
        Converts Drone objects to dictionary format for visualization while keeping
        Building objects intact for direct property access.
        """
        self.time = time
        self.drones = [{
            'id': drone.id,
            'position': drone.position.tolist(),
            'velocity': drone.velocity.tolist(),
            'flight_path': [w.tolist() for w in drone.flight_path.waypoints],
            'current_waypoint_index': drone.flight_path.current_index,
            'successful': drone.successful,
            'status': drone.status
        } for drone in drones]
        self.buildings = buildings
        self.drone_collisions = drone_collisions
        self.building_collisions = building_collisions