import numpy as np
import logging

from drone_sim.core.system import BaseSystem
from drone_sim.core.events import EVENT_NAMES
from drone_sim.models.drone import Drone, DroneStatus

logger = logging.getLogger(__name__)


class MovementSystem(BaseSystem):
    """Handles drone movement physics without complex simulations"""
    
    def __init__(self, dependencies=None, engine=None):
        super().__init__(dependencies or {})
        self.engine = engine
        self.event_bus = self.dependencies.get('event_bus')
    
    def update(self, dt: float):
        """Update movement for all drones"""
        # Get drones directly from drone manager
        drones = self.engine.systems['drones'].active_drones
        
        for drone in drones:
            try:
                # Skip inactive drones
                if drone.status != DroneStatus.ACTIVE:
                    logger.debug("Skipping inactive drone %d (status: %s)",
                               drone.id, drone.status)
                    continue
                
                # Get max speed from model
                if not hasattr(drone.model, 'max_speed_m_s'):
                    logger.error("Drone %d missing max_speed_m_s attribute", drone.id)
                    continue
                
                # Get current waypoint and calculate 3D direction
                current_waypoint = drone.flight_path.current_waypoint
                direction = current_waypoint - drone.position
                distance = np.linalg.norm(direction)
                
                logger.debug("Drone %d @ %s moving to waypoint %s (3D distance: %.2f m)",
                           drone.id, drone.position, current_waypoint, distance)
                
                # Check if we've reached the waypoint
                if drone.flight_path.is_nearing_waypoint(drone.position):
                    logger.debug("Drone %d reached waypoint %d @ %s",
                               drone.id, drone.flight_path.current_target_index, drone.position)
                    
                    # Get next waypoint before advancing
                    next_waypoint = drone.flight_path.next_waypoint
                    if not drone.flight_path.advance_waypoint():
                        logger.info("Drone %d completed path at %s", drone.id, drone.position)
                        drone.status = DroneStatus.SUCCESSFUL
                        continue
                    
                    logger.debug("Drone %d advancing to next waypoint @ %s",
                               drone.id, next_waypoint)
                    # Recalculate direction to new waypoint
                    current_waypoint = drone.flight_path.current_waypoint
                    direction = current_waypoint - drone.position
                    distance = np.linalg.norm(direction)
                
                # Calculate and set velocity
                if distance > 0:
                    # Calculate 3D unit vector and scale by max speed
                    unit_vector = direction / distance
                    max_total_speed = drone.model.max_speed_m_s
                    
                    # Calculate horizontal and vertical components separately
                    horizontal_speed = np.linalg.norm(unit_vector[:2]) * max_total_speed
                    max_ascent = getattr(drone.model, 'max_ascent_rate', 5.0)  # Default 5 m/s
                    max_descent = getattr(drone.model, 'max_descent_rate', 3.0)  # Default 3 m/s
                    vertical_speed = np.clip(unit_vector[2] * max_total_speed,
                                            -max_descent, 
                                            max_ascent)
                    
                    # Normalize horizontal components while preserving direction
                    if horizontal_speed > 0:
                        horizontal_direction = unit_vector[:2] / np.linalg.norm(unit_vector[:2])
                    else:
                        horizontal_direction = np.zeros(2)
                    
                    velocity = np.array([
                        horizontal_direction[0] * horizontal_speed,
                        horizontal_direction[1] * horizontal_speed,
                        vertical_speed
                    ])
                    
                    # Log detailed movement info
                    logger.debug("Drone %d movement calculation:", drone.id)
                    logger.debug("  Direction: %s (distance: %.2f m)", direction, distance)
                    logger.debug("  Unit vector: %s", unit_vector)
                    logger.debug("  Velocity: %s (speed: %.2f m/s)",
                               velocity, np.linalg.norm(velocity))
                    
                    # Add after velocity calculation
                    logger.debug("Velocity components - X: %.2f, Y: %.2f, Z: %.2f", 
                               velocity[0], velocity[1], velocity[2])
                    logger.debug("Speed breakdown - Horizontal: %.2f m/s, Vertical: %.2f m/s",
                               np.linalg.norm(velocity[:2]), abs(velocity[2]))
                    
                    # Update velocity
                    prev_velocity = drone.velocity
                    drone.velocity = velocity
                    
                    if not np.array_equal(prev_velocity, velocity):
                        logger.debug("  Velocity changed: %s -> %s", prev_velocity, velocity)
                else:
                    drone.velocity = np.zeros(3)
                    logger.debug("Drone %d at waypoint, velocity zeroed", drone.id)
                
                # Update position if drone is moving
                if np.any(drone.velocity != 0):
                    prev_pos = drone.position.copy()
                    drone.move(dt)
                    
                    # Calculate and log movement details
                    movement = drone.position - prev_pos
                    movement_distance = np.linalg.norm(movement)
                    
                    if movement_distance > 0:
                        logger.debug("Drone %d movement:", drone.id)
                        logger.debug("  Distance: %.2f m in %.3fs", movement_distance, dt)
                        logger.debug("  Components - dX: %.2f, dY: %.2f, dZ: %.2f",
                                   movement[0], movement[1], movement[2])
                        logger.debug("  Speed: %.2f m/s", movement_distance / dt)
                        logger.debug("  Status: %s", drone.status.name)
                
            except Exception as e:
                logger.error("Failed to update drone %d: %s", drone.id, e)
        
        # Add state snapshot logging
        logger.debug("Post-movement state - Drones: %s", 
                   [(d.id, d.position.tolist()) for d in drones])
        
        updated_drones = drones
        
        logger.debug("Movement output - first drone position: %s", 
                   updated_drones[0].position if updated_drones else "None")

        # Publish movement update
        if self.event_bus:
            drone_states = [d.to_dict() for d in updated_drones]
            self.event_bus.publish(
                EVENT_NAMES['MOVEMENT_UPDATED'],
                {'drones': drone_states, 'timestamp': self.engine.time}
            )
        
        return updated_drones 