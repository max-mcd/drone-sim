from typing import List

from drone_sim.models.building import Building
from drone_sim.models.city_data import CityData

from drone_sim.systems.environment.city_loader import CityLoader


class TerrainSystem:
    def __init__(self):
        self.height_map = None
        self.noise_profile = None
        
    def configure(self, config: dict):
        """Configure terrain parameters"""
        self.height_map = config.get('height_map')
        self.noise_profile = config.get('noise_profile')
        
    def set_city(self, city_name: str):
        """Update terrain for specific city"""
        pass  # Implement city-specific terrain setup
        
    def get_buildings(self) -> List[Building]:
        return self.current_city.buildings if self.current_city else [] 