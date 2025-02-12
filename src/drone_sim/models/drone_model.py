import json
import logging
from pathlib import Path

logger = logging.getLogger(__name__)

class DroneModel:
    """Represents a specific drone model's physical and performance characteristics.
    
    This class encapsulates all the key specifications and capabilities of a drone model:
    - Physical dimensions (length, width, height)
    - Performance limits (speed, altitude, range)
    - Safety parameters (deceleration, safety buffers)
    - Operating specs (payload capacity, battery life)
    
    The model serves as a template for creating drone instances in the simulation,
    ensuring consistent behavior based on real-world limitations.
    
    Safety features:
    - Calculates minimum safety buffers for collision avoidance
    - Accounts for braking distance, response time, and physical size
    - Enforces maximum speed and altitude limits
    """
    # Safety buffer calculation constants
    SAFETY_CALCULATION = {
        'response_time': 0.5,        # Time to respond to potential collision (seconds)
        'physical_buffer_multiplier': 1.5,  # Multiplier for physical dimensions
        'base_separation': 15.0      # Minimum separation distance (meters)
    }

    REQUIRED_FIELDS = [
        'name',
        'max_speed_m_s', 
        'max_altitude_m', 
        'range_km',
        'battery_life_hours',
        'payload_capacity_kg',
        'max_ascent_rate',
        'max_descent_rate'
    ]
    # Default acceleration/deceleration values if not specified
    DEFAULT_ACCELERATION = 5.0  # m/s^2
    DEFAULT_DECELERATION = 3.0  # m/s^2

    def __init__(self, 
                 name: str,
                 max_speed_m_s: float,
                 max_altitude_m: float,
                 range_km: float,
                 battery_life_hours: float,
                 payload_capacity_kg: float,
                 max_ascent_rate: float,
                 max_descent_rate: float,
                 length_m: float = 1.2,
                 width_m: float = 1.0,
                 height_m: float = 0.4,
                 acceleration_m_s2: float = None,
                 deceleration_m_s2: float = None):
        """Initialize drone model from configuration data"""
        # Validate config completeness
        for field in self.REQUIRED_FIELDS:
            if field not in locals():
                raise ValueError(f"Missing required field: {field}")
        self.name = name
        self.max_speed_m_s = max_speed_m_s
        self.max_altitude_m = max_altitude_m
        self.range_km = range_km
        self.battery_life_hours = battery_life_hours
        self.acceleration_m_s2 = acceleration_m_s2 or self.DEFAULT_ACCELERATION
        self.deceleration_m_s2 = deceleration_m_s2 or self.DEFAULT_DECELERATION
        logger.debug(f"""
            Initializing drone model {self.name}:
            Max speed: {self.max_speed_m_s} m/s
            Max altitude: {self.max_altitude_m} m
        """)
        self.max_deceleration = self.deceleration_m_s2
        self.length_m = length_m
        self.width_m = width_m
        self.height_m = height_m
        self.payload_capacity_kg = payload_capacity_kg
        self.battery_capacity_s = battery_life_hours * 3600
        self.max_ascent_rate = max_ascent_rate
        self.max_descent_rate = max_descent_rate

    def calculate_safety_buffer(self, current_speed: float) -> float:
        """Calculate buffer using actual speed instead of max speed"""
        braking_distance = (current_speed ** 2) / (2 * self.deceleration_m_s2)
        response_buffer = current_speed * self.SAFETY_CALCULATION['response_time']
        physical_size = max(self.length_m, self.width_m)
        return braking_distance + response_buffer + (physical_size * 1.5)

    # Default vertical movement rates
    DEFAULT_MAX_ASCENT_RATE = 5.0  # m/s
    DEFAULT_MAX_DESCENT_RATE = 3.0  # m/s

    @classmethod
    def load_preset(cls, model_name: str):
        """Load model definition from data directory"""
        try:
            models_path = Path(__file__).parent.parent.parent.parent / 'data/drones/drone_models.json'
            logger.debug(f"Loading drone model {model_name} from {models_path}")
            
            with open(models_path) as f:
                all_models = json.load(f)
                model_data = all_models["drone_models"][model_name]
                logger.debug(f"Loaded model data for {model_name}: {model_data}")
                
                return cls(
                    name=model_data['name'],
                    max_speed_m_s=model_data['max_speed_m_s'],
                    max_altitude_m=model_data['max_altitude_m'],
                    range_km=model_data['range_km'],
                    battery_life_hours=model_data['battery_life_hours'],
                    payload_capacity_kg=model_data['payload_capacity_kg'],
                    max_ascent_rate=model_data.get('max_ascent_rate', cls.DEFAULT_MAX_ASCENT_RATE),
                    max_descent_rate=model_data.get('max_descent_rate', cls.DEFAULT_MAX_DESCENT_RATE),
                    length_m=model_data['dimensions_m'][0],
                    width_m=model_data['dimensions_m'][1],
                    height_m=model_data['dimensions_m'][2],
                    acceleration_m_s2=model_data.get('acceleration_m_s2'),
                    deceleration_m_s2=model_data.get('deceleration_m_s2')
                )
        except Exception as e:
            logger.error(f"Failed to load drone model {model_name}: {str(e)}")
            raise