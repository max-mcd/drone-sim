from dataclasses import dataclass
from typing import Dict


@dataclass
class Building:
    x: float
    y: float
    height: float
    width: float
    length: float

    def to_dict(self) -> Dict:
        """Convert building state to dictionary"""
        return {
            'id': getattr(self, 'id', 0),
            'x': self.x,
            'y': self.y,
            'width': self.width,
            'length': self.length,
            'height': self.height
        }