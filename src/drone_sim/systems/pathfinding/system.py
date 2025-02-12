import logging
import time

import numpy as np

from drone_sim.core.events import EVENT_NAMES, EventBus
from drone_sim.exceptions import PathfindingError
from drone_sim.models.drone import Drone, DroneStatus
from drone_sim.models.protocols import Updatable
from drone_sim.systems.base import BaseSystem

logger = logging.getLogger(__name__)

class PathfindingSystem(BaseSystem):
    """Handles drone path following and waypoint navigation"""
    
    def __init__(self, dependencies):
        super().__init__(dependencies)
        self.event_bus = dependencies.get('event_bus')
        self.failed_drones = []
        self.drone_manager = dependencies.get('drone_manager')
        
    def configure(self, config: dict) -> None:
        """Configure pathfinding parameters"""
        # Add configuration logic here
        self.config = config
        
    def update(self, dt: float) -> None:
        """Update paths for all active drones"""
        if not self.drone_manager:
            return
            
        for drone in self.drone_manager.drones:
            if drone.status == DroneStatus.ACTIVE:
                self.recalculate_path(drone)

    def recalculate_path(self, drone: Drone):
        current_waypoint = drone.flight_path.current_waypoint
        direction = current_waypoint - drone.position
        distance = np.linalg.norm(direction)
        
        if distance > 0:
            # Calculate ideal velocity vector
            ideal_velocity = (direction / distance) * drone.model.max_speed_m_s
            # Remove the velocity mixing code and just set target direction
            drone.target_direction = ideal_velocity  # MovementSystem will handle actual acceleration

    def avoid_collision(self, drone: Drone, collision_point: np.ndarray):
        """Generate new waypoint around collision"""
        direction = collision_point - drone.position
        if np.linalg.norm(direction) > 0:
            new_waypoint = collision_point + direction * 50.0
            drone.flight_path.insert_waypoint(new_waypoint)
            logger.info(f"Drone {drone.id} path replanned around {collision_point}") 

    def complete_route(self, drone: Drone):
        self.event_bus.publish(
            EVENT_NAMES['PATH_COMPLETED'],
            {'drone_id': drone.id}
        )
        logger.info(f"Drone {drone.id} reached final waypoint") 

    def replan_failed_routes(self):
        """Attempt to replan routes for drones with failed paths"""
        for drone in self.failed_drones:
            try:
                new_path = self.plan_route(drone)
                drone.flight_path = new_path
            except PathfindingError as e:
                logger.error(f"Replan failed for drone {drone.id}: {e}") 