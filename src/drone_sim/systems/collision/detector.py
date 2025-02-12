import logging
from collections import defaultdict
from itertools import combinations
from typing import List, Tuple
import time

import numpy as np

from drone_sim.core.events import EventBus, EVENT_NAMES
from drone_sim.models.building import Building
from drone_sim.models.drone import Drone
from drone_sim.systems.collision.history import CollisionHistory
from drone_sim.models.collision import CollisionRecord

logger = logging.getLogger(__name__)

class CollisionDetector:
    """Simplified collision detection with spatial partitioning"""
    
    def __init__(self, grid_resolution: float = 50.0):
        self.grid_resolution = grid_resolution
        self.event_bus = EventBus()
        self.history = CollisionHistory()
        logger.info(f"Collision detector initialized with {grid_resolution}m grid")

    def find_collisions(self, drones: List[Drone], buildings: List[Building]) -> List[CollisionRecord]:
        """Main detection entry point - now returns collision records"""
        collidables = drones + buildings
        grid = self._create_spatial_grid(collidables)
        potential_pairs = self._get_potential_pairs(grid)
        
        collisions = []
        for a, b in potential_pairs:
            if self._check_pair(a, b):
                collisions.append(self._create_collision_record(a, b))
        
        for collision in collisions:
            self.event_bus.publish(EVENT_NAMES['COLLISION_DETECTED'], {
                'record': collision.to_dict(),
                'sim_time': time.time()  # Should use simulation clock
            })
        
        return collisions

    def _create_spatial_grid(self, collidables: list) -> dict:
        """3D spatial partitioning to reduce checks"""
        grid = defaultdict(list)
        for obj in collidables:
            cell = (
                int(obj.position[0] // self.grid_resolution),
                int(obj.position[1] // self.grid_resolution),
                int(obj.position[2] // self.grid_resolution)
            )
            grid[cell].append(obj)
        return grid

    def _get_potential_pairs(self, grid: dict) -> list:
        """Generate candidate pairs from grid cells"""
        return [
            (a, b) 
            for cell in grid.values() 
            for a, b in combinations(cell, 2)
            if len(cell) > 1
        ]

    def _check_pair(self, a, b) -> bool:
        """Unified collision check for different pair types"""
        if isinstance(a, Drone) and isinstance(b, Drone):
            return self._drone_collision(a, b)
        if isinstance(a, Building) or isinstance(b, Building):
            drone, building = (a, b) if isinstance(a, Drone) else (b, a)
            return self._building_collision(drone, building)
        return False

    def _drone_collision(self, drone1: Drone, drone2: Drone) -> bool:
        """Drone-to-drone collision check"""
        distance = np.linalg.norm(drone1.position - drone2.position)
        return distance < (drone1.collision_radius + drone2.collision_radius)

    def _building_collision(self, drone: Drone, building: Building) -> bool:
        """Drone-to-building collision check"""
        return (abs(drone.position[0] - building.x) < building.width/2 and
                abs(drone.position[1] - building.y) < building.length/2 and
                drone.position[2] < building.height)

    def _create_collision_record(self, a, b) -> CollisionRecord:
        """Create standardized collision record"""
        timestamp = time.time()  # Use simulation time in real implementation
        position = self._get_collision_position(a, b)
        
        if isinstance(a, Drone) and isinstance(b, Drone):
            return CollisionRecord.from_drone_collision(
                a.id, b.id, timestamp, position
            )
        else:  # Building collision
            drone, building = (a, b) if isinstance(a, Drone) else (b, a)
            return CollisionRecord.from_building_collision(
                drone.id, building.id, timestamp, position
            )

    def _get_collision_position(self, a, b) -> tuple:
        """Calculate midpoint between colliding objects"""
        if isinstance(a, Building):
            return (a.x, a.y, a.height)  # Building collision at roof level
        return tuple((a.position + b.position) / 2) 