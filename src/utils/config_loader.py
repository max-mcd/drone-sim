import json
from typing import Dict

from ..models.drone import DroneModel


def load_city_data(city_data_path: str, city_name: str) -> dict:
    """Load city parameters from cities data file
    
    Args:
        city_data_path: Path to cities-data.json
        city_name: Name of the city to load
        
    Returns:
        dict containing city parameters:
            - building_density: buildings per km²
            - avg_height: average building height in meters
            - population_density: people per km²
            - takeoff_landing_locations: number of takeoff/landing points
            
    Raises:
        ValueError: If city_name not found in cities data
    """
    with open(city_data_path) as f:
        data = json.load(f)
        if city_name not in data['cities']:
            raise ValueError(f"City {city_name} not found in cities data")
        
        return data['cities'][city_name]


def load_simulation_config(path: str) -> dict:
    with open(path) as f:
        data = json.load(f)
        
        # Convert coordinate lists to dicts for each drone
        for drone in data['drones']:
            drone['start'] = {
                'x': drone['start'][0],
                'y': drone['start'][1],
                'z': drone['start'][2]
            }
            drone['destination'] = {
                'x': drone['destination'][0],
                'y': drone['destination'][1],
                'z': drone['destination'][2]
            }
        return data


def load_drone_models(path: str) -> Dict[str, DroneModel]:
    with open(path) as f:
        data = json.load(f)
    return {
        name: DroneModel(model_data)
        for name, model_data in data['drone_models'].items()
    }