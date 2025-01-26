import yaml
import json
from typing import Dict
from ..models.drone_model import DroneModel

def load_simulation_config(path: str) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)

def load_drone_models(path: str) -> Dict[str, DroneModel]:
    with open(path) as f:
        data = json.load(f)
        return {
            name: DroneModel(
                max_speed=model['max_speed'],
                range=model['range'],
                dimensions=model['dimensions'],
                payload_capacity=model['payload_capacity'],
                battery_life=model['battery_life']
            )
            for name, model in data['drone_models'].items()
        }