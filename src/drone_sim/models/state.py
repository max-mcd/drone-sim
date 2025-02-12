from dataclasses import dataclass
from typing import List
import numpy as np

@dataclass
class SimulationState:
    time: float
    drones: list
    buildings: list
    collisions: list

    def __init__(self, time: float, drones: list, buildings: list, collisions: list):
        self.time = time
        self.drones = drones
        self.buildings = buildings
        self.collisions = collisions

    def to_dict(self):
        """Convert all entities to serialized dicts for visualization/export"""
        return {
            'time': self.time,
            # Handle both object and dict representations
            'drones': [d.to_dict() if hasattr(d, 'to_dict') else d for d in self.drones],
            'buildings': [b.to_dict() if hasattr(b, 'to_dict') else b for b in self.buildings],
            'collisions': [c.to_dict() if hasattr(c, 'to_dict') else c for c in self.collisions]
        } 