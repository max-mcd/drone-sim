from dataclasses import dataclass
from typing import List, Tuple

from .building import Building


@dataclass
class CityData:
    name: str
    building_density: int
    avg_height: float
    population_density: int
    takeoff_locations: int
    buildings: List[Building]
    dimensions: Tuple[float, float, float]