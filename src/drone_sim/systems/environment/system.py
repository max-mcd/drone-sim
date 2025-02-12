from drone_sim.core.base_system import BaseSystem
from drone_sim.models.city_data import CityData
from .weather import DummyWeatherSystem
from drone_sim.systems.environment.city_loader import CityLoader
import logging

logger = logging.getLogger(__name__)


class EnvironmentSystem(BaseSystem):
    def __init__(self, dependencies=None, sim_config=None):
        super().__init__(dependencies or {})
        self.sim_config = sim_config or {}
        self._current_city = None
        self.weather_system = DummyWeatherSystem()
        
        # Get validated path from simulation config
        self.cities_data_path = self.sim_config['cities_data_path']
        
        # Initialize loader with required parameters
        self.loader = CityLoader(
            data_path=self.cities_data_path,
            config=self.sim_config
        )
        logger.debug("Initialized EnvironmentSystem with config: %s", self.sim_config)
        
    def configure(self, config: dict):
        """Handle configuration updates"""
        city_name = config.get('city', {}).get('name')  # Extract name from city config
        if not city_name:
            raise ValueError("Missing city name in environment configuration")
        
        if not self._current_city or self._current_city.name != city_name:
            # Load city through loader which calculates building count
            self._current_city = self.loader.load_city(city_name)
            
            logger.info("Loaded city: %s with %d buildings", 
                      self._current_city.name, 
                      len(self.get_buildings()))

    def get_buildings(self):
        """Get all buildings in current city"""
        if self._current_city and self.loader:
            return self.loader.get_buildings(self._current_city.name)
        return []

    def update(self, dt: float):
        """Update environmental factors"""
        pass  # Implement weather updates if needed

    @property
    def current_city(self) -> CityData:
        """Get current city data"""
        return self._current_city

    @current_city.setter
    def current_city(self, value: CityData) -> None:
        """Set current city data"""
        self._current_city = value 

    def load_city(self, city_config: dict):
        """Load a city configuration"""
        self._current_city = CityLoader.load_from_config(city_config)
        logger.info("Loaded city: %s with %d buildings", 
                  self._current_city.name, 
                  len(self._current_city.buildings)) 