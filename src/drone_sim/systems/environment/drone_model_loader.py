import json

class DroneModelLoader:
    def __init__(self, data_path: str):
        self.data_path = data_path
        
    def load_models(self) -> dict:
        with open(self.data_path) as f:
            return json.load(f)['drone_models'] 