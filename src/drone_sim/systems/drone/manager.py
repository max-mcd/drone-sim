from typing import List

from drone_sim.core.events import EventBus
from drone_sim.models.drone import Drone, DroneStatus
from drone_sim.utils.config_loader import get_drone_model
from drone_sim.core.base_system import BaseSystem

import logging
import numpy as np

logger = logging.getLogger(__name__)


class DroneManager(BaseSystem):
    def __init__(self, dependencies=None):
        super().__init__(dependencies or {})
        self.next_id = 1
        self.event_bus = self.dependencies.get('event_bus')
        self._drones = {}
        self._active_drones = set()
        
    def setup(self, config: list):
        """Initialize drones from config"""
        for i, drone_config in enumerate(config):
            full_config = {
                'id': i,
                'model': drone_config['model'],
                'waypoints': drone_config['waypoints'],
                'priority': drone_config.get('priority', 0),
                'emergency': drone_config.get('is_emergency', False)
            }
            self.add_drone(full_config)
            
        logger.info(f"Initialized {len(self._drones)} drones")
        
    def update(self, dt: float):
        """Update drone states (except movement)"""
        for drone in self._drones.values():
            if drone.status == DroneStatus.ACTIVE:
                # Update non-movement state (battery, status checks, etc)
                self._update_drone_state(drone, dt)
            
    def _update_drone_state(self, drone: Drone, dt: float):
        """Update drone state without handling movement"""
        # Update battery level
        if hasattr(drone, 'battery_level'):
            drone.battery_level -= dt / drone.model.battery_capacity_s
        
        # Check for completed flight using current_target_index
        if hasattr(drone, 'flight_path'):
            # Only mark as landed if we've actually reached the final position
            if drone.status == DroneStatus.SUCCESSFUL:
                drone.status = DroneStatus.LANDED
                logger.info("Drone %d completed flight path and landed", drone.id)

        # Add altitude-based status check
        if drone.position[2] < 10:  # Minimum safe altitude
            drone.status = DroneStatus.LANDED
            logger.info(f"Drone {drone.id} safely landed @ {drone.position}")

    def add_drone(self, drone_config: dict):
        """Add a new drone to the simulation"""
        drone_id = drone_config['id']
        model_name = drone_config['model']
        
        try:
            model = get_drone_model(model_name)
            logger.debug("Loaded drone model %s with attributes: %s",
                       model_name, model.__dict__)
            
            # Verify required attributes
            required_attrs = ['max_speed_m_s', 'max_altitude_m', 'range_km',
                             'battery_life_hours', 'payload_capacity_kg',
                             'max_ascent_rate', 'max_descent_rate']
            for attr in required_attrs:
                if not hasattr(model, attr):
                    logger.error("Model %s missing required attribute: %s", model_name, attr)
                    return
            
            # Create actual Drone instance
            new_drone = Drone(
                drone_id=drone_id,
                model=model,
                waypoints=drone_config['waypoints'],
                priority=drone_config.get('priority', 0),
                is_emergency=drone_config.get('emergency', False)
            )
            
            self._drones[drone_id] = new_drone
            self._active_drones.add(drone_id)
            logger.info("Added drone %d (%s) with max_speed=%.1f m/s",
                       drone_id, model.name, model.max_speed_m_s)
            
        except Exception as e:
            logger.error("Failed to create drone %d with model %s: %s",
                       drone_id, model_name, e, exc_info=True)

    @property
    def drones(self) -> list[Drone]:
        """Get all drone objects as a list"""
        return list(self._drones.values())

    @property
    def active_drones(self) -> list[Drone]:
        """Get list of active Drone objects"""
        return [d for d in self._drones.values() if d.status == DroneStatus.ACTIVE]

    def get_all_drones(self) -> list[dict]:
        """Get serialized state for all drones"""
        return [drone.to_dict() for drone in self._drones.values()]

    def get_drone_state(self, drone_id: int) -> dict:
        """Get serialized state for a single drone"""
        drone = self._drones[drone_id]
        return {
            'id': drone.id,
            'position': drone.position.tolist(),
            'velocity': drone.velocity.tolist(),
            'status': drone.status.value
        }