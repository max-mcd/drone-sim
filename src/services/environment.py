import json
from typing import List, Tuple

import numpy as np

from ..models.building import Building
from ..models.city_data import CityData


class Environment:
    def __init__(self, city_data_path: str, dimensions: Tuple[float, float, float]):
        self.dimensions = dimensions
        with open(city_data_path, 'r') as f:
            data = json.load(f)
            self.cities = {
                name: self._generate_city(city_data, name)
                for name, city_data in data['cities'].items()
            }
        self.current_city = None

    def _generate_city(self, city_data: dict, city_name: str) -> CityData:
        """Generate a city from the provided parameters"""
        buildings = self._generate_buildings(
            int(city_data['building_density_per_km2']),
            float(city_data['avg_height_m']),
            self.dimensions
        )
        return CityData(
            name=city_name,
            building_density=int(city_data['building_density_per_km2']),
            avg_height=float(city_data['avg_height_m']),
            population_density=int(city_data['population_density_per_km2']),
            takeoff_locations=int(city_data['takeoff_landing_locations_count']),
            buildings=buildings
        )

    def _generate_buildings(self, density: int, avg_height: float, dimensions: Tuple[float, float, float]) -> List[Building]:
        """Building generation uses different probability distributions to model realistic city layouts:

        1. Uniform distribution (x,y coordinates): Buildings are placed randomly across the city area
           with equal probability. This models cities with grid-like layouts where buildings can be
           located anywhere within the boundaries.

        2. Normal/Gaussian distribution (height): Building heights follow a bell curve centered around
           the average height with standard deviation of 20% of the mean. This models the tendency of
           most buildings to cluster around a typical height, with fewer very tall or very short buildings.

        3. Uniform distribution (width/length): Building footprints are randomly sized between 10-30m.
           This range represents typical building sizes while maintaining simplicity.

        Args:
            density: Number of buildings per square km (1-100)
            avg_height: Average building height in meters
            dimensions: (width, length, height) of city area in meters

        Returns:
            List[Building]: List of randomly generated buildings with properties:
                - x,y: Uniformly distributed coordinates within city dimensions
                - height: Normally distributed around avg_height with 20% std dev
                - width,length: Uniformly distributed between 10-30m

        Raises:
            ValueError: If density < 0 or dimensions contains negative values
        """
        if density < 0:
            raise ValueError("Building density must be non-negative")
        if any(d <= 0 for d in dimensions):
            raise ValueError("City dimensions must be positive")
        
        num_buildings = int((dimensions[0] * dimensions[1] / 1e6) * density)
        buildings = []
        
        for _ in range(num_buildings):
            x = np.random.uniform(0, dimensions[0])
            y = np.random.uniform(0, dimensions[1])
            height = np.random.normal(avg_height, avg_height * 0.2)
            width = np.random.uniform(10, 30)
            length = np.random.uniform(10, 30)
            
            buildings.append(Building(x, y, height, width, length))
        
        return buildings

    def set_city(self, city_name: str) -> None:
        """Set the current city by name"""
        if city_name not in self.cities:
            raise ValueError(f"City {city_name} not found")
        self.current_city = self.cities[city_name]