from typing import Dict, List, Tuple, Optional
import csv
import numpy as np
from ..models.city_data import CityData
from ..models.building import Building

class Environment:
    def __init__(self, city_data_path: str, dimensions: Tuple[float, float, float]):
        self.dimensions = dimensions
        self.cities: Dict[str, CityData] = self._load_city_data(city_data_path)
        self.current_city: Optional[CityData] = None

    def _load_city_data(self, path: str) -> Dict[str, CityData]:
        """Load city data from CSV file and generate buildings.

        Args:
            path: Path to CSV file containing city data

        Returns:
            Dict mapping city names to CityData objects with generated buildings

        Raises:
            FileNotFoundError: If CSV file does not exist
            KeyError: If required columns are missing from CSV
        """

        cities = {}
        with open(path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                # Generate random buildings based on density
                buildings = self._generate_buildings(
                    int(row['building_density']),
                    float(row['avg_height']),
                    self.dimensions
                )
                cities[row['city_name']] = CityData(
                    name=row['city_name'],
                    building_density=int(row['building_density']),
                    avg_height=float(row['avg_height']),
                    population_density=int(row['population_density']),
                    takeoff_locations=int(row['takeoff_landing_locations']),
                    buildings=buildings
                )
        return cities

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
        """Set the current city for the simulation environment.
        
        This method updates the current_city attribute to the city specified by city_name.
        The city must exist in the environment's cities dictionary.
        
        Args:
            city_name: str - Name of the city to set as current
            
        Raises:
            ValueError: If the specified city name is not found in the available cities
        """
        self.current_city = self.cities.get(city_name)
        if not self.current_city:
            raise ValueError(f"City {city_name} not found")