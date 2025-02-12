from .base import AbstractStrategy


class ProbabilisticAvoidance(AbstractStrategy):
    def __init__(self, config):
        self.threshold = config['risk_threshold']  # 0.65 from YAML
        
    def resolve(self, collisions):
        for collision in collisions:
            drone1, drone2 = collision.participants
            risk = self._calculate_risk(drone1, drone2)
            
            if risk > self.threshold:
                # Apply avoidance to higher risk drone
                avoidance_velocity = self._calculate_avoidance(drone1, drone2)
                drone1.apply_avoidance_velocity(avoidance_velocity)  # Defined in Drone class 