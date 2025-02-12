import numpy as np
from drone_sim.core.base_system import BaseSystem
from drone_sim.core.simulation_state import SimulationState
from drone_sim.models.drone import DroneStatus
from drone_sim.core.events import EVENT_NAMES
from drone_sim.models.collision import CollisionRecord

import logging

logger = logging.getLogger(__name__)

class StateManager(BaseSystem):
    def __init__(self, dependencies=None):
        super().__init__(dependencies or {})
        self.error_log = []
        self.event_bus = None
        self.engine = None  # Will be set via dependencies
        self._current_state = None

    def configure(self, config: dict):
        """Configure after dependencies are injected"""
        self.event_bus = self.dependencies.get('event_bus')
        self.engine = self.dependencies.get('engine')
        
        if not self.engine:
            raise RuntimeError("StateManager requires engine reference")
            
        # Initialize state reference
        self._current_state = self.engine.state
        
        # Setup event subscriptions AFTER engine is available
        if self.event_bus:
            logger.info("Subscribing to collision events")
            self.event_bus.subscribe(
                EVENT_NAMES['COLLISION_DETECTED'], 
                self.handle_collision
            )
            self.event_bus.subscribe(
                EVENT_NAMES['MOVEMENT_UPDATED'], 
                self.handle_movement_update
            )

    def update(self, dt: float):
        """Update state from all systems"""
        try:
            drone_manager = self.engine.systems['drones']
            env_system = self.engine.systems['environment']
            collision_system = self.engine.systems['collision']
            
            # Update state with fresh data
            self._current_state.time = self.engine.time
            self._current_state.drones = [d.to_dict() for d in drone_manager.drones]
            self._current_state.buildings = [b.to_dict() for b in env_system.current_city.buildings] if env_system.current_city else []
            self._current_state.collisions = collision_system.get_recent_records()
            
            # Log only active drones
            active_drones = [d for d in self._current_state.drones if d['status'] == DroneStatus.ACTIVE.value]
            if active_drones:
                logger.debug("Active drones at t=%.1f:", self.engine.time)
                for drone in active_drones:
                    logger.debug("  Drone %d @ (%.1f, %.1f, %.1f)",
                              drone['id'],
                              drone['position'][0],
                              drone['position'][1],
                              drone['position'][2])
            
            # Always publish state after update
            if self.event_bus:
                state_dict = self.current_state.to_dict()
                logger.info("Publishing state update @ t=%.1f with %d drones", 
                           self.engine.time, 
                           len(state_dict['drones']))
                
                # Log first drone position for debugging
                if state_dict['drones']:
                    logger.debug("Sample drone position: %s", 
                               state_dict['drones'][0].get('position', 'N/A'))
                    
                self.event_bus.publish(
                    EVENT_NAMES['STATE_UPDATED'], 
                    state_dict
                )
                logger.debug("State update published successfully")
                
        except Exception as e:
            logger.error("State update failed: %s", e, exc_info=True)

    def record_error(self, error: dict):
        """Record error details"""
        self.error_log.append({
            'timestamp': self.engine.time,
            **error
        })
        # Could publish event directly via self.event_bus if needed

    @property
    def current_state(self) -> SimulationState:
        return self._current_state 

    def update_state(self):
        """Called by engine during simulation loop"""
        drone_manager = self.engine.systems['drones']
        env_system = self.engine.systems.get('environment')
        
        # Safely get city buildings
        city_buildings = []
        if env_system and env_system.current_city:
            city_buildings = [b.to_dict() for b in env_system.current_city.buildings]
        
        # Get raw drone objects instead of dicts
        drones = drone_manager.drones  # Instead of get_all_drones()
        
        # Create new state with all required data
        new_state = SimulationState(
            time=self.engine.current_time,  # Use current_time property
            drones=drones,  # Pass objects directly
            buildings=city_buildings,
            collisions=self.engine.systems['collision'].get_recent_records()
        )
        
        # Log only active drones and their positions
        active_drones = [d for d in new_state.drones if d.status == DroneStatus.ACTIVE]
        if active_drones:
            logger.info("Active Drones at %.1fs:", new_state.time)
            for drone in active_drones:
                logger.info("  Drone %d @ (%.1f, %.1f, %.1f)",
                          drone.id,
                          drone.position[0],
                          drone.position[1],
                          drone.position[2])
        
        # Add null check before publishing
        if self.event_bus:
            state_dict = new_state.to_dict()
            # Only log essential info, not the full building data
            logger.debug("State Summary - Time: %.1fs, Drones: %d, Active Buildings: %d",
                        state_dict['time'],
                        len(state_dict['drones']),
                        len(state_dict['buildings']))
            self.event_bus.publish(
                EVENT_NAMES['STATE_UPDATED'],
                state_dict
            )
        else:
            logger.error("No event bus available for state publishing!")

    def get_drone_state(self, drone_id: int) -> dict:
        drone = self.drones[drone_id]
        return {
            'id': drone.id,
            'x': drone.position[0],  # Add explicit position fields
            'y': drone.position[1],
            'z': drone.position[2],
            'status': drone.status.name,
            'velocity': drone.velocity.tolist(),
            'battery': drone.battery_level,
            'path_progress': drone.flight_path.progress,
            'flight_path': {
                'waypoints': [wp.tolist() for wp in drone.flight_path.waypoints],
                'current_waypoint_index': drone.flight_path.current_target_index
            }
        }

    def get_all_drones(self) -> list[dict]:
        return [self.get_drone_state(did) for did in self.drones] 

    def handle_movement_update(self, event_data: dict):
        """Update drone positions from movement system events"""
        logger.debug("Processing movement update: %s", event_data.keys())
        try:
            drones = event_data['drones']  # These are already serialized dicts
            timestamp = event_data['timestamp']
            
            # Update drone states in current_state while preserving existing state
            if not hasattr(self._current_state, 'drones'):
                self._current_state.drones = []
            
            # Create a map of existing drones for quick lookup
            existing_drones = {d['id']: d for d in self._current_state.drones}
            
            # Update or add new drone states
            current_drones = []
            for drone_state in drones:
                drone_id = drone_state['id']
                if drone_id in existing_drones:
                    # Update existing drone state while preserving additional fields
                    updated_state = existing_drones[drone_id].copy()
                    updated_state.update(drone_state)
                    current_drones.append(updated_state)
                else:
                    current_drones.append(drone_state)
            
            self._current_state.drones = current_drones
            logger.debug("Updated state with %d drone positions", len(current_drones))
            self._current_state.time = timestamp
            
            logger.debug("Updated state with %d drone positions", len(drones))

        except KeyError as e:
            logger.error("Invalid movement event format: %s", str(e))

    def handle_collision(self, event_data: dict):
        """Process collision events into state with receipt logging"""
        logger.debug("Received collision event: %s", event_data.keys())
        
        # Add sequence validation
        if 'timestamp' not in event_data:
            logger.warning("Collision event missing timestamp")
            return
        
        try:
            # Create collision record from event data
            record = CollisionRecord(
                participants=(event_data['drone_ids'] if event_data['type'] == 'drone' 
                             else (event_data['drone_id'], event_data['building_id'])),
                timestamp=self.engine.time,
                collision_type=event_data['type'],
                position=event_data.get('position', [])
            )
            
            self.current_state.collisions.append(record)
            logger.info("Added collision to state: %s", record)
            
            # Publish state update with collision context
            self.event_bus.publish(
                EVENT_NAMES['STATE_UPDATED'],
                {
                    **self.current_state.to_dict(),
                    'last_collision': record.to_dict()
                }
            )
            
        except KeyError as e:
            logger.error("Invalid collision event format: %s", str(e)) 