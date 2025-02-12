import time
from typing import Dict
import logging

logger = logging.getLogger(__name__)


class SimulationState:
    """Centralized state management for the simulation engine.
    
    Now properly located in core module per architecture diagram.
    """
    time: float
    drones: list
    buildings: list
    collisions: list
    collision_metrics: dict
    movement_metrics: dict
    
    def __init__(self, time: float = 0.0, drones: list = None, buildings: list = None, 
                 collisions: list = None, movement_metrics: dict = None, version: int = 1):
        self.version = version
        self._snapshots: dict = {}
        self.time = time
        self.drones = drones or []
        self.buildings = buildings or []
        self.collisions = collisions or []
        self.collision_metrics = {
            'avoidance_maneuvers': 0,
            'path_replans': 0,
            'total_collisions': 0
        }
        self.movement_metrics = movement_metrics or {'max_speed': 0}
        
    def update_from_systems(self, movement_system, collision_system, pathfinding_system):
        """Update state from various systems"""
        # Get time from engine instead of movement system
        self.time = movement_system.engine.time
        # Get drones from drone manager system
        self.drones = [d.to_dict() for d in movement_system.engine.systems['drones'].drones]
        self.collisions = collision_system.get_recent_records()

    def create_snapshot(self):
        self.version += 1
        self._snapshots[self.version] = self.__dict__.copy() 

    def to_dict(self):
        state_dict = {
            'time': self.time,
            'drones': [
                d if isinstance(d, dict) else d.to_dict() 
                for d in self.drones
            ],
            'buildings': [
                b if isinstance(b, dict) else b.to_dict() 
                for b in self.buildings
            ],
            'collisions': [
                c if isinstance(c, dict) else c.to_dict() 
                for c in self.collisions
            ],
            'collision_metrics': self.collision_metrics,
            'movement_metrics': self.movement_metrics
        }
        logger.debug("Serialized state keys: %s", state_dict.keys())  # Verify structure
        return state_dict 