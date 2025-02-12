from dataclasses import dataclass
from typing import Dict

import numpy as np

from drone_sim.models.protocols import Collidable


@dataclass
class Building(Collidable):
    id: int
    x: float  # Bottom-left X (matches city_loader's placement)
    y: float  # Bottom-left Y (matches city_loader's placement)
    height: float
    width: float
    length: float

    def __init__(self, x: float, y: float, width: float, length: float, height: float, building_id: int = 0):
        self.id = building_id
        self.x = x
        self.y = y
        self.width = width
        self.length = length
        self.height = height

    @property
    def x1(self) -> float:
        return self.x
    
    @property
    def x2(self) -> float:
        return self.x + self.width
    
    @property
    def y1(self) -> float:
        return self.y
    
    @property
    def y2(self) -> float:
        return self.y + self.length

    def to_dict(self) -> Dict:
        """Convert building state to dictionary"""
        return {
            'id': self.id,
            'x': self.x,
            'y': self.y,
            'width': self.width,
            'length': self.length,
            'height': self.height
        }

    @property
    def position(self) -> np.ndarray:
        return np.array([self.x, self.y, self.height/2])
    
    @property
    def collision_radius(self) -> float:
        return max(self.width, self.length) / 2
    
    def predict_position(self, time: float) -> np.ndarray:
        return self.position.copy()

    def to_geojson(self) -> dict:
        return {
            "type": "Feature",
            "geometry": {
                "type": "Polygon",
                "coordinates": [[
                    [self.x, self.y],
                    [self.x + self.width, self.y],
                    [self.x + self.width, self.y + self.length],
                    [self.x, self.y + self.length],
                    [self.x, self.y]
                ]]
            },
            "properties": {
                "height": self.height,
                "id": self.id
            }
        }