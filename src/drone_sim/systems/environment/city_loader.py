import json
from typing import List

import numpy as np

from drone_sim.models.building import Building
from drone_sim.models.city_data import CityData


class CityLoader:
    def __init__(self, data_path: str, config: dict):
        self.data_path = data_path
        self.config = config
        self.city_cache = {}  # Add cache for loaded cities
        
    def configure(self, config: dict):
        """Handle configuration updates"""
        self.config.update(config)
        self.city_cache.clear()  # Clear cache on config changes
        
    def get_buildings(self, city_name: str) -> list:
        """Get generated buildings for a city"""
        if city_name not in self.city_cache:
            self.load_city(city_name)
        return self.city_cache[city_name].buildings

    def load_city(self, city_name: str) -> CityData:
        """Load city with density-based building calculation"""
        if city_name in self.city_cache:
            return self.city_cache[city_name]
            
        with open(self.data_path) as f:
            cities_data = json.load(f)
            city_info = cities_data['cities'][city_name]
            
            # Corrected config access - use dimensions directly
            area_km2 = (self.config['dimensions']['x'] / 1000) * \
                      (self.config['dimensions']['y'] / 1000)
            building_count = int(city_info['building_density_per_km2'] * area_km2)
            
            city = CityData(
                name=city_name,
                dimensions=(
                    self.config['dimensions']['x'],
                    self.config['dimensions']['y'],
                    self.config['dimensions']['z']
                ),
                buildings=self._generate_buildings(city_info, building_count)
            )
            self.city_cache[city_name] = city
            return city
            
    def _generate_buildings(self, city_info: dict, count: int) -> list:
        """Generate random buildings with realistic dimensions using specified distributions"""
        avg_height = city_info['avg_height_m']
        city_width = self.config['dimensions']['x']
        city_length = self.config['dimensions']['y']
        
        buildings = []
        for _ in range(count):
            # Position - uniform distribution across city area
            x = np.random.uniform(0, city_width)
            y = np.random.uniform(0, city_length)
            
            # Dimensions - uniform between 10-30m
            width = np.random.uniform(10, 30)
            length = np.random.uniform(10, 30)
            
            # Height - normal distribution around average height
            height = np.random.normal(avg_height, avg_height * 0.2)
            height = max(height, 15)  # Ensure minimum height of 15m
            
            # Ensure buildings stay within city boundaries
            x = min(x, city_width - width)
            y = min(y, city_length - length)
            
            buildings.append(Building(
                x=x,
                y=y,
                width=width,
                length=length,
                height=height,
                building_id=len(buildings)
            ))
        return buildings 