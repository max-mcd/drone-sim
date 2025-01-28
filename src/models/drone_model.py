class DroneModel:
    def __init__(self, model_data: dict):
        self.max_speed = model_data['max_speed_m_s']
        self.range = model_data['range_km'] * 1000  # Convert to meters
        self.dimensions = {
            'length': model_data['dimensions_m'][0],
            'width': model_data['dimensions_m'][1],
            'height': model_data['dimensions_m'][2]
        }
        self.payload_capacity = model_data['payload_capacity_kg']
        self.battery_life = model_data['battery_life_hours'] 