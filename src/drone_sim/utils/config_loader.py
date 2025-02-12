from typing import Dict
import json

import numpy as np

from drone_sim.models.drone_model import DroneModel
from drone_sim.utils.config_validator import validate_config_file

import logging
logger = logging.getLogger(__name__)

# Add at the top with other module-level variables
_DRONE_MODELS = None

def load_simulation_config(path: str) -> dict:
    """Load and validate simulation configuration from JSON file"""
    global _DRONE_MODELS
    config_data = validate_config_file(path, 'simulation_config')
    
    # Add default paths if not specified
    sim_config = config_data.setdefault('simulation', {})
    sim_config.setdefault('cities_data_path', 'data/cities/cities.json')
    sim_config.setdefault('drone_models_path', 'data/drones/drone_models.json')
    
    # Convert waypoint lists to FlightPath objects
    for drone in config_data['drones']:
        drone['flight_path'] = [
            np.array(waypoint) for waypoint in drone['waypoints']
        ]
    
    # Pass full config data to validation
    validate_drone_positions(config_data)
    
    # Load drone models after config validation
    models_path = sim_config['drone_models_path']
    _DRONE_MODELS = load_drone_models(models_path)  # Store loaded models
    
    return config_data


def load_drone_models(path: str) -> Dict[str, DroneModel]:
    """Load drone models from JSON file and create DroneModel objects"""
    with open(path) as f:
        data = json.load(f)
    
    return {
        name: DroneModel(
            name=model_data['name'],
            max_speed_m_s=model_data['max_speed_m_s'],
            max_altitude_m=model_data['max_altitude_m'],
            range_km=model_data['range_km'],
            battery_life_hours=model_data['battery_life_hours'],
            length_m=model_data['dimensions_m'][0],  # Extract from array
            width_m=model_data['dimensions_m'][1],
            height_m=model_data['dimensions_m'][2],
            payload_capacity_kg=model_data['payload_capacity_kg'],
            max_ascent_rate=model_data.get('max_ascent_rate', 5.0),
            max_descent_rate=model_data.get('max_descent_rate', 3.0)
        )
        for name, model_data in data['drone_models'].items()
    }


def validate_drone_positions(config_data: dict):
    """Validate drone starting positions and waypoints"""
    # Get airspace dimensions from config
    airspace_x = config_data['simulation']['dimensions']['x']
    airspace_y = config_data['simulation']['dimensions']['y']
    
    start_positions = {}
    for i, drone_config in enumerate(config_data['drones']):
        # Check minimum waypoints
        if len(drone_config['waypoints']) < 2:
            raise ValueError(f"Drone {i} must have at least 2 waypoints")
            
        # Load drone model to check altitude limits
        model_name = drone_config['model']
        try:
            drone_model = DroneModel.load_preset(model_name)
            max_altitude = drone_model.max_altitude_m
        except Exception as e:
            raise ValueError(f"Failed to load model {model_name} for drone {i}: {e}")
            
        # Check unique starting positions
        start_pos = tuple(drone_config['waypoints'][0])
        if start_pos in start_positions:
            raise ValueError(
                f"Drones {i} and {start_positions[start_pos]} "
                f"share starting position {start_pos}"
            )
        start_positions[start_pos] = i
        
        # Check waypoint boundaries
        for wp in drone_config['waypoints']:
            if wp[0] < 0 or wp[0] > airspace_x:
                raise ValueError(f"Drone {i} waypoint X {wp[0]} out of bounds (0-{airspace_x})")
            if wp[1] < 0 or wp[1] > airspace_y:
                raise ValueError(f"Drone {i} waypoint Y {wp[1]} out of bounds (0-{airspace_y})")
            if wp[2] < 0 or wp[2] > max_altitude:
                raise ValueError(
                    f"Drone {i} waypoint altitude {wp[2]}m exceeds model {model_name} "
                    f"limits (0-{max_altitude}m)"
                )


def get_drone_model(model_name: str) -> DroneModel:
    """Get a DroneModel instance for the specified model name"""
    # Load from JSON using DroneModel's load_preset
    try:
        return DroneModel.load_preset(model_name)
    except Exception as e:
        logger.error(f"Failed to load drone model {model_name}: {e}")
        raise  # Re-raise the exception to handle it in the caller


def load_drone_model(model_name: str) -> DroneModel:
    """Alias for get_drone_model (deprecated)"""
    return get_drone_model(model_name)


# Removed duplicate get_drone_model function