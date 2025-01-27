import yaml
import json
from typing import Dict
from ..models.drone import DroneModel

def load_simulation_config(path: str) -> dict:
    with open(path) as f:
        data = json.load(f)
        # Convert dimensions list to dict format
        data['simulation']['dimensions'] = {
            'x': data['simulation']['dimensions'][0],
            'y': data['simulation']['dimensions'][1],
            'z': data['simulation']['dimensions'][2]
        }
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
            name: DroneModel(
                max_speed=model['max_speed_m_s'],
                range=model['range_km'] * 1000,  # Convert km to meters
                dimensions={
                    'length': model['dimensions_m'][0],  # Convert from list to dict
                    'width': model['dimensions_m'][1],
                    'height': model['dimensions_m'][2]
                },
                payload_capacity=model['payload_capacity_kg'],
                battery_life=model['battery_life_hours']
            )
            for name, model in data['drone_models'].items()
        }