class DummyWeatherSystem:
    """Non-functional weather system for API compatibility"""
    def configure(self, config: dict):
        pass
        
    def update(self, dt: float):
        pass

    @property
    def current_conditions(self) -> dict:
        return {
            'temperature': 20.0,
            'wind_speed': 0.0,
            'precipitation': 0.0,
            'status': 'clear'
        } 