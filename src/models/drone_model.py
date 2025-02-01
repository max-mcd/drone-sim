import logging

# Get logger for this module using the module's name
logger = logging.getLogger(__name__)

class DroneModel:
    # Safety buffer calculation constants
    SAFETY_CALCULATION = {
        'response_time': 0.5,        # Time to respond to potential collision (seconds)
        'physical_buffer_multiplier': 1.5,  # Multiplier for physical dimensions
        'base_separation': 15.0      # Minimum separation distance (meters)
    }

    def __init__(self, model_data: dict):
        """Initialize drone model from configuration data"""
        self.name = model_data['name']
        self.max_speed = model_data['max_speed_m_s']
        self.max_altitude = model_data['max_altitude_m']
        logger.debug(f"""
            Initializing drone model {self.name}:
            Max speed: {self.max_speed} m/s
            Max altitude: {self.max_altitude} m
        """)
        self.max_deceleration = model_data.get('max_deceleration', self.max_speed * 2)  # Default to 2x max_speed
        self.range = model_data['range_km'] * 1000  # Convert to meters
        self.dimensions = {
            'length': model_data['dimensions_m'][0],
            'width': model_data['dimensions_m'][1],
            'height': model_data['dimensions_m'][2]
        }
        self.payload_capacity = model_data['payload_capacity_kg']
        self.battery_life = model_data['battery_life_hours'] 

    def calculate_safety_buffer(self) -> float:
        """Calculate minimum safety distance based on drone capabilities"""
        # Buffer components:
        # 1. Braking distance (v²/2a) - distance needed to stop
        # 2. Response time buffer (v * t) - distance covered during system response
        # 3. Physical size buffer - based on drone dimensions
        # 4. Base separation - minimum safe distance
        
        braking_distance = (self.max_speed ** 2) / (2 * self.max_deceleration)
        response_buffer = self.max_speed * self.SAFETY_CALCULATION['response_time']
        physical_size = max(self.dimensions['length'], self.dimensions['width'])
        base_separation = self.SAFETY_CALCULATION['base_separation']
        
        total_buffer = (braking_distance + 
                       response_buffer + 
                       physical_size * self.SAFETY_CALCULATION['physical_buffer_multiplier'] + 
                       base_separation)
        
        logger.debug(f"""
            Safety buffer calculation for {self.name}:
            Braking distance: {braking_distance:.1f}m
            Response buffer: {response_buffer:.1f}m
            Physical buffer: {physical_size:.1f}m
            Base separation: {base_separation:.1f}m
            Total buffer: {total_buffer:.1f}m
        """)
        
        return total_buffer 