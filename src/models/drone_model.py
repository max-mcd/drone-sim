from dataclasses import dataclass
from typing import Dict

@dataclass
class DroneModel:
    max_speed: float
    range: float
    dimensions: Dict[str, float]
    payload_capacity: float
    battery_life: float