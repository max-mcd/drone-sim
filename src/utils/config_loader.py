import json
from typing import Dict, List

import numpy as np

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


def validate_drone_positions(drone_configs: List[dict]) -> None:
    """Validate drone configurations
    
    Checks:
    1. No two drones share the same starting position
    2. Each drone has at least 2 waypoints
    """
    start_positions = {}
    for i, config in enumerate(drone_configs):
        # Validate waypoint count
        if len(config['waypoints']) < 2:
            raise ValueError(f"Drone {i} must have at least 2 waypoints")
            
        # Validate unique starting positions
        start_pos = tuple(config['waypoints'][0])
        if start_pos in start_positions:
            raise ValueError(f"Invalid configuration: Drone {i} and {start_positions[start_pos]} share starting position {start_pos}")
        start_positions[start_pos] = i


def load_simulation_config(path: str) -> dict:
    """Load and validate simulation configuration from JSON file"""
    with open(path) as f:
        data = json.load(f)
        
    # Convert waypoint lists to FlightPath objects
    for drone in data['drones']:
        drone['flight_path'] = [
            np.array(waypoint) for waypoint in drone['waypoints']
        ]
    
    return data


def load_drone_models(path: str) -> Dict[str, DroneModel]:
    with open(path) as f:
        data = json.load(f)
    return {
        name: DroneModel(model_data)
        for name, model_data in data['drone_models'].items()
    }