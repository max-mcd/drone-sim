import logging
from typing import Callable, Dict
import time
import threading
from pathlib import Path
import traceback
from pprint import pformat

from jsonschema import ValidationError

from drone_sim.core.events import EVENT_NAMES
from drone_sim.exceptions import CollisionError, PhysicsError, SimulationError
from drone_sim.models.protocols import Updatable

from drone_sim.systems import (
    DroneManager,
    EnvironmentSystem,
    PathfindingSystem,
)
from drone_sim.systems.movement.system import MovementSystem
from drone_sim.utils.config_validator import validate_config, validate_config_file
from drone_sim.core.events import EventBus
from drone_sim.models.protocols import Configurable
from drone_sim.systems.visualization.matplotlib import MatplotlibVisualizer
from drone_sim.systems.collision import CollisionSystem
from drone_sim.systems.pathfinding import PathfindingSystem
from drone_sim.systems.state import StateManager
from drone_sim.models.state import SimulationState

logger = logging.getLogger(__name__)

class SimulationEngine:
    """Orchestrates simulation systems and manages execution lifecycle"""
    
    @classmethod
    def from_config(cls, config_path: str) -> 'SimulationEngine':
        """Create engine instance from config file path"""
        from drone_sim.utils.config_loader import load_simulation_config
        config = load_simulation_config(config_path)
        return cls(config)

    def __init__(self, config: dict, event_bus: EventBus = None):
        validate_config(config, 'simulation_config')
        self.config = config
        
        # Add simulation timing parameters
        self.time_step = config['simulation'].get('time_step', 0.1)
        self.max_duration = config['simulation'].get('max_duration_sec', 300)
        self.real_time_factor = config['simulation'].get('real_time_factor', 1.0)
        
        # Phase 1: Initialize core infrastructure
        self.event_bus = event_bus or EventBus()
        self.systems = {}
        self._running = False
        self.time = 0.0
        self._simulation_time = 0.0
        
        # Initialize simulation state
        self.state = SimulationState(
            time=0.0,
            drones=[],
            buildings=[],
            collisions=[]
        )

        # Phase 2: Create system instances with empty dependencies
        systems = {
            'event_bus': self.event_bus,
            'state': StateManager(dependencies={}),
            'environment': EnvironmentSystem(
                dependencies={},  # Empty initial deps
                sim_config=config['simulation']
            ),
            'drones': DroneManager(dependencies={}),
            'movement': MovementSystem(dependencies={}),
            'visualization': MatplotlibVisualizer(dependencies={}),
            'pathfinding': PathfindingSystem(dependencies={}),
            'collision': CollisionSystem(dependencies={})
        }

        # Phase 3: Update dependency injection
        systems['state'].dependencies.update({
            'event_bus': self.event_bus,
            'engine': self
        })

        systems['environment'].dependencies.update({
            'event_bus': self.event_bus,
            'cities_data_path': config['simulation'].get('cities_data_path')
        })

        # Add validation to ensure path exists
        if not Path(config['simulation']['cities_data_path']).exists():
            raise ValueError(f"Invalid cities data path: {config['simulation']['cities_data_path']}")

        systems['drones'].dependencies.update({
            'event_bus': self.event_bus,
            'environment': systems['environment']
        })

        systems['movement'].dependencies.update({
            'event_bus': self.event_bus,
            'drones': systems['drones']
        })
        systems['movement'].engine = self  # Temporary bridge until full DI

        systems['visualization'].dependencies.update({
            'event_bus': self.event_bus,
            'engine': self,
            'config': config
        })

        systems['pathfinding'].dependencies.update({
            'drone_manager': systems['drones'],
            'event_bus': self.event_bus,
            'environment': systems['environment']
        })

        systems['collision'].dependencies.update({
            'pathfinding': systems['pathfinding'],
            'event_bus': self.event_bus,
            'engine': self,
            'drones': systems['drones'],
            'environment': systems['environment']
        })

        # Phase 4: Finalize system registry
        self.systems = systems
        self._visualizer_ref = systems['visualization']
        
        # Phase 5: Configure systems
        for name, system in self.systems.items():
            if isinstance(system, Configurable):
                if name == 'environment':
                    # Pass city configuration from simulation config
                    system_config = {
                        'city': {
                            'name': self.config['simulation']['city']
                        }
                    }
                else:
                    system_config = self.config.get('systems', {}).get(name, {})
                system.configure(system_config)
        
        # Phase 6: Initialize systems in proper order
        self._init_systems()
        self.initialize_systems()
        
    def _init_systems(self):
        """Initialize all registered systems"""
        for name, system in self.systems.items():
            if hasattr(system, 'setup'):
                system.setup(self.config.get(name, {}))
            self._connect_system_events(system)
        
    def _connect_system_events(self, system):
        """Centralized system event wiring"""
        # Generic protocol-based connections
        if hasattr(system, 'on_collision'):
            self.event_bus.subscribe('COLLISION', system.on_collision)
        if hasattr(system, 'on_physics_update'):
            self.event_bus.subscribe('PHYSICS_UPDATE', system.on_physics_update)
        
        # System-specific connections
        if isinstance(system, StateManager):
            self.event_bus.subscribe(
                EVENT_NAMES['MOVEMENT_UPDATED'],
                system.handle_movement_update
            )
            self.event_bus.subscribe(
                EVENT_NAMES['COLLISION_DETECTED'],
                system.handle_collision
            )
        elif isinstance(system, MatplotlibVisualizer):
            self.event_bus.subscribe(
                EVENT_NAMES['STATE_UPDATED'],
                system.on_state_update
            )
        
    def add_hook(self, phase: str, callback: Callable):
        """Add custom logic to simulation phases"""
        self.hooks[phase] = callback
        
    def run(self):
        """Main simulation loop with corrected time tracking"""
        logger.info("Simulation starting with config: %s", self.config)
        logger.debug("Running in thread: %s", threading.current_thread().name)
        self._running = True
        last_update = time.time()
        
        try:
            while self._running:
                current_time = time.time()
                dt = current_time - last_update
                
                if dt >= self.time_step:
                    # Concise drone position logging
                    logger.debug("=== Simulation Time: %.1fs ===", self._simulation_time)
                    active_drones = self.systems['drones'].active_drones
                    logger.debug("Active Drones: %d/%d",
                               len(active_drones),
                               len(self.systems['drones'].drones))
                    
                    # Log positions in a table format
                    if active_drones:
                        logger.debug("Drone Positions:")
                        logger.debug("ID    |    X    |    Y    |    Z    | Status")
                        logger.debug("-" * 45)
                        for drone in active_drones:
                            logger.debug("%2d    | %7.1f | %7.1f | %7.1f | %s",
                                       drone.id,
                                       drone.position[0],
                                       drone.position[1],
                                       drone.position[2],
                                       drone.status.label)
                    
                    # Update both simulation time properties
                    self._simulation_time += self.time_step
                    self.time = self._simulation_time
                    
                    # Update systems with fixed time step
                    self._update_systems(self.time_step)
                    last_update = current_time
                    
                    # Log visualization state
                    logger.debug("Visualization Status - Active: %s",
                                self._running)
                    
                self.systems['state'].update_state()
                
                # Only sleep if we're running in real-time mode
                if self.real_time_factor > 0:
                    sleep_time = max(0, (self.time_step / self.real_time_factor) - dt)
                    if sleep_time > 0:
                        time.sleep(sleep_time)
                
        except Exception as e:
            logger.error("Simulation failed: %s", traceback.format_exc())
        finally:
            self.shutdown()
        
    def _pre_run(self):
        """Pre-simulation setup"""
        self._running = True
        self.time = 0.0
        self.event_bus.publish(
            EVENT_NAMES['SIMULATION_START'],
            {'time': self.time}
        )
        
        # Execute custom pre-run hooks
        if 'pre_run' in self.hooks:
            self.hooks['pre_run']()
        
    def _post_run(self):
        """Cleanup after simulation"""
        self.event_bus.publish(
            EVENT_NAMES['SIMULATION_END'],
            {'time': self.time}
        )
        for system in self.systems.values():
            if hasattr(system, 'teardown'):
                system.teardown()
        
    def _update_systems(self, dt: float):
        """Update all systems in proper order with enhanced logging"""
        try:
            logger.debug("\n=== System Update Cycle ===")
            logger.debug("Starting update at t=%.1f (dt=%.3f)", self.current_time, dt)
            
            # Update systems in specific order
            update_order = ['environment', 'drones', 'movement', 'collision', 'state', 'visualization']
            
            for system_name in update_order:
                if system_name not in self.systems:
                    logger.warning("System '%s' not found in registered systems", system_name)
                    continue
                    
                system = self.systems[system_name]
                try:
                    logger.debug(">>> Updating %s system", system_name)
                    start_time = time.time()
                    
                    # Log pre-update state for key systems
                    if system_name == 'drones':
                        logger.debug("Pre-update drone count: %d", len(system.drones))
                    elif system_name == 'visualization':
                        logger.debug("Pre-update visualization state: figure=%s",
                                   hasattr(system, 'figure'))
                    
                    # Perform update
                    system.update(dt)
                    
                    # Log post-update state
                    update_time = time.time() - start_time
                    logger.debug("<<< %s system updated (took %.3fs)",
                               system_name, update_time)
                    
                    if update_time > 0.1:  # Log warning for slow updates
                        logger.warning("%s system update took %.3fs",
                                     system_name, update_time)
                        
                except Exception as e:
                    logger.error("Error updating %s system: %s",
                               system_name, str(e), exc_info=True)
                    logger.debug("System state at failure: %s",
                               vars(system))
            
            # Publish updated state
            try:
                logger.debug("Publishing state update...")
                start_time = time.time()
                self._publish_state()
                logger.debug("State published (took %.3fs)",
                           time.time() - start_time)
            except Exception as e:
                logger.error("State publishing failed: %s", str(e), exc_info=True)
                
            logger.debug("=== Update Cycle Complete ===\n")
            
        except Exception as e:
            logger.error("Simulation update failed: %s", e, exc_info=True)

    def _check_stop_conditions(self):
        """Check global completion conditions"""
        return False  # TODO: Implement actual checks
        
    def _validate_systems(self):
        required_protocols = {
            'collision': [Updatable, Configurable],
            'pathfinding': [Updatable]
        }
        
        for system_name, protocols in required_protocols.items():
            system = self.systems[system_name]
            for protocol in protocols:
                if not isinstance(system, protocol):
                    raise TypeError(f"{system_name} must implement {protocol.__name__}") 

    def generate_report(self) -> dict:
        return {
            'avoided_collisions': [
                r.to_dict() for r in self.systems['collision'].history.records
                if r.avoided
            ],
            'collisions': self.systems['collision'].history.get_summary(),
            'metrics': self.state_manager.get_metrics(),
            'duration': self.time
        } 

    def shutdown(self):
        """Cleanly terminate simulation"""
        self._running = False
        # Add any cleanup logic here
        if hasattr(self, 'visualizer'):
            self.visualizer.close()
        logging.info("Simulation shutdown complete") 

    def get_current_state(self) -> SimulationState:
        """Get current simulation state with proper time tracking"""
        return SimulationState(
            time=self.current_time,  # Use dedicated time tracking
            drones=self.systems['drones'].drones,
            buildings=self.systems['environment'].current_city.buildings,
            collisions=self.systems['collision'].get_recent_records()
        )

    @property
    def current_time(self) -> float:
        """Get current simulation time from central clock"""
        return self._simulation_time

    @current_time.setter
    def current_time(self, value: float):
        """Set current simulation time"""
        self._simulation_time = value

    def initialize_systems(self):
        # Connect collision system to state manager
        self.systems['collision'].event_bus = self.event_bus
        # Connect movement system
        pass

        # Inject event bus into all systems
        for name, system in self.systems.items():
            if hasattr(system, 'event_bus'):
                system.event_bus = self.event_bus

    def _publish_state(self):
        """Update and publish current state"""
        self.state = self.get_current_state()  # Update the state
        state_dict = self.state.to_dict()
        logger.debug("Publishing state with %d drones, %d buildings, time=%.1f",
                   len(state_dict['drones']), 
                   len(state_dict['buildings']),
                   state_dict['time'])
        
        self.event_bus.publish(
            EVENT_NAMES['STATE_UPDATED'],
            state_dict
        )

    @property
    def current_state(self) -> SimulationState:
        """Get current simulation state"""
        return self.state