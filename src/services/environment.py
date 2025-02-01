from typing import List, Tuple

import numpy as np

from ..utils.config_loader import load_city_data
from ..models.building import Building
from ..models.city_data import CityData


class Environment:
    """Environment manages the simulation's physical environment and conditions.
    
    This class is responsible for:
    - Loading and managing city data and layouts
    - Generating and placing buildings based on city parameters
    - Managing environmental conditions that affect drone flight
    
    The Environment class could be extended to include additional environmental factors such as:
    
    Weather Conditions:
    - Wind speed and direction (affects drone stability and energy usage)
    - Precipitation (reduces visibility and affects sensors)
    - Temperature (impacts battery performance and motor efficiency)
    - Air pressure (affects lift and flight characteristics)
    - Visibility conditions (fog, smog affecting sensor performance)
    
    Time-based Factors:
    - Time of day (affecting visibility and traffic patterns)
    - Seasonal variations (temperature, daylight hours)
    - Solar radiation (affecting solar-powered drones)
    
    Urban Environment:
    - RF interference zones (affecting drone communications)
    - No-fly zones (restricted airspace)
    - Temporary obstacles (construction cranes, event structures)
    - Bird activity zones (risk of wildlife collisions)
    - Thermal updrafts from buildings
    
    These extensions would allow for more realistic simulation of drone operations
    in varying environmental conditions, enabling better testing of drone control
    systems and flight planning algorithms.
    """
    def __init__(self, city_data_path: str, dimensions: Tuple[float, float, float]):
        self.dimensions = dimensions
        self.city_data_path = city_data_path
        self.current_city = None

    def set_city(self, city_name: str) -> None:
        """Set the current city configuration"""
        city_data = load_city_data(self.city_data_path, city_name)
        self.current_city = CityData(
            name=city_name,
            building_density=city_data['building_density_per_km2'],
            avg_height=city_data['avg_height_m'],
            population_density=city_data['population_density_per_km2'],
            takeoff_locations=city_data['takeoff_landing_locations_count'],
            buildings=self._generate_buildings(
                density=city_data['building_density_per_km2'],
                avg_height=city_data['avg_height_m'],
                dimensions=self.dimensions
            ),
            dimensions=self.dimensions
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
        print(f"Generating {num_buildings} buildings for density {density}/km²")  # Debug print
        buildings = []
        
        for _ in range(num_buildings):
            x = np.random.uniform(0, dimensions[0])
            y = np.random.uniform(0, dimensions[1])
            height = np.random.normal(avg_height, avg_height * 0.2)
            width = np.random.uniform(10, 30)
            length = np.random.uniform(10, 30)
            
            buildings.append(Building(x, y, height, width, length))
        
        return buildings