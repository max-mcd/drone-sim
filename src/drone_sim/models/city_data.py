from dataclasses import dataclass
from typing import List, Tuple

from drone_sim.models.building import Building


@dataclass
class CityData:
    name: str
    building_density_per_km2: int
    avg_height_m: float
    population_density_per_km2: int
    takeoff_locations: int
    buildings: List[Building]
    dimensions: Tuple[float, float, float]

    def __init__(self, name: str, buildings: list[Building], dimensions: Tuple[float, float, float]):
        self.name = name
        self.buildings = buildings
        self.dimensions = dimensions

    def add_building(self, building):
        self.buildings.append(building)

    def to_geojson(self) -> dict:
        """Convert city data to GeoJSON format for visualization"""
        return {
            "type": "FeatureCollection",
            "features": [
                {
                    "type": "Feature",
                    "geometry": {
                        "type": "Polygon",
                        "coordinates": [[
                            [0, 0], 
                            [self.dimensions[0], 0],
                            [self.dimensions[0], self.dimensions[1]],
                            [0, self.dimensions[1]],
                            [0, 0]
                        ]]
                    },
                    "properties": {
                        "name": self.name,
                        "type": "city_boundary"
                    }
                },
                *[b.to_geojson() for b in self.buildings]
            ]
        }

    def to_dict(self) -> dict:
        """Serialization for state management"""
        return {
            "name": self.name,
            "building_density_per_km2": self.building_density_per_km2,
            "avg_height_m": self.avg_height_m,
            "population_density_per_km2": self.population_density_per_km2,
            "takeoff_locations": self.takeoff_locations,
            "buildings": [b.to_dict() for b in self.buildings],
            "dimensions": self.dimensions
        }