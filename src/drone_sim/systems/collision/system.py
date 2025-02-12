import importlib

from drone_sim.core.system import BaseSystem
from drone_sim.utils.config_validator import validate_config
from drone_sim.systems.collision.history import CollisionHistory
from drone_sim.models.collision import CollisionRecord
from drone_sim.core.events import EventBus
from .strategies import NoOpAvoidance
from drone_sim.models.drone import DroneStatus
from drone_sim.systems.collision.detector import CollisionDetector
from drone_sim.core.events import EVENT_NAMES

import logging
import numpy as np

logger = logging.getLogger(__name__)


class CollisionSystem(BaseSystem):
    def __init__(self, dependencies):
        super().__init__(dependencies)
        self.engine = None  # Will be set in configure()
        self.detector = CollisionDetector()
        self.history = CollisionHistory()
        self.strategies = {}
        self.active_strategy = NoOpAvoidance()
        self.event_bus = None  # Will be set in configure()
        
    def configure(self, config: dict):
        """Proper dependency injection"""
        super().configure(config)  # Let BaseSystem handle config first
        self.engine = self.dependencies.get('engine')
        self.event_bus = self.dependencies.get('event_bus')
        
        if not self.engine:
            raise RuntimeError("CollisionSystem requires engine reference")
        if not self.event_bus:
            raise RuntimeError("CollisionSystem requires event bus reference")
        
        # Merge FIRST, validate AFTER
        self.config = {
            'grid_resolution': 50,
            'strategies': {
                'active': 'hierarchical',
                'params': {
                    # Add default parameters for hierarchical strategy
                    'hierarchical': {
                        'priority_mode': 'altitude_based',
                        'min_speed_factor': 0.3,
                        'max_speed_factor': 0.7,
                        'replan_distance': 50.0
                    }
                }
            },
            **config.get('collision', {})
        }
        
        # Ensure active strategy has at least empty config
        active_strat = self.config['strategies']['active']
        if active_strat not in self.config['strategies']['params']:
            self.config['strategies']['params'][active_strat] = {}
        
        validate_config(self.config, 'collision_system')
        
        self._load_strategies()
        self.active_strategy = self.strategies.get(
            self.config['strategies']['active'], 
            NoOpAvoidance()
        )

    def _load_strategies(self):
        strategies_module = importlib.import_module('.strategies', package=__package__)
        
        # Load all strategies specified in config params
        for name, params in self.config['strategies']['params'].items():
            try:
                class_name = name.capitalize() + 'Avoidance'
                strategy_class = getattr(strategies_module, class_name)
                self.strategies[name] = strategy_class(**params)
            except AttributeError:
                logger.warning(f"Strategy {name} not found, using NoOp")
                self.strategies[name] = NoOpAvoidance()

    def resolve_collisions(self, collisions: list[CollisionRecord]):
        """Handle collision resolution with null checks"""
        for collision in collisions:
            if collision.avoided:
                continue
            
            if collision.collision_type == 'drone':
                drone1 = self._get_drone_by_id(collision.drone_id)
                drone2 = self._get_drone_by_id(collision.other_id)
                if drone1 and drone2:  # Add null checks
                    self._handle_drone_collision(drone1, drone2, collision.position)
                else:
                    logger.warning(f"Skipping drone collision - missing participant(s) "
                                 f"D1:{collision.drone_id} D2:{collision.other_id}")
            
            elif collision.collision_type == 'building':
                drone = self._get_drone_by_id(collision.drone_id)
                building = self._get_building_by_id(collision.building_id)
                if drone and building:  # Add null checks
                    self._handle_building_collision(drone, building)
                else:
                    logger.warning(f"Skipping building collision - missing "
                                 f"Drone:{collision.drone_id} Building:{collision.building_id}")
            
            collision.avoided = True
            self.history.add_record(collision)

    def _handle_drone_collision(self, drone1, drone2, position):
        """Handle drone-drone collision resolution"""
        if not drone1 or not drone2:
            logger.error("Invalid drone collision participants")
            return
        
        if drone1.status == DroneStatus.ACTIVE:
            drone1.status = DroneStatus.COLLIDED
        if drone2.status == DroneStatus.ACTIVE:
            drone2.status = DroneStatus.COLLIDED
        
        logger.warning(f"Collision between D{drone1.id} and D{drone2.id} at {position}")
        
        # Convert position safely for different types
        if isinstance(position, (np.ndarray, list, tuple)):
            pos_list = list(position)
        else:
            pos_list = []
            logger.warning("Invalid position type in collision")
        
        self.event_bus.publish(
            EVENT_NAMES['COLLISION_DETECTED'],
            {
                'type': 'drone',
                'drone_ids': [drone1.id, drone2.id],
                'position': pos_list,
                'timestamp': self.engine.time
            }
        )

    def _handle_building_collision(self, drone, building):
        """Safer building collision handling"""
        if not drone or not building:
            logger.error("Invalid collision participants")
            return
        
        if drone.status == DroneStatus.ACTIVE:
            drone.status = DroneStatus.COLLIDED
            logger.warning(f"Drone {drone.id} collided with building {building.id}")
        
        # Convert position safely    
        if hasattr(drone.position, '__iter__'):
            pos_list = list(drone.position)
        else:
            pos_list = []
            logger.warning("Invalid drone position type")
        
        self.event_bus.publish(
            EVENT_NAMES['COLLISION_DETECTED'],
            {
                'type': 'building',
                'drone_id': drone.id,
                'building_id': building.id,
                'position': pos_list,
                'timestamp': self.engine.time
            }
        )

    def get_recent_records(self):
        """Get recent collision records from history"""
        return self.history.get_recent_records()

    def update(self, dt: float):
        """Main collision detection update with entity validation"""
        # Get fresh entity references each update
        env_system = self.engine.systems.get('environment')
        drone_system = self.engine.systems.get('drones')
        
        if not env_system or not drone_system or not env_system.current_city:
            logger.warning("Skipping collision update - missing dependencies")
            return
        
        # Get current state copies to avoid mid-update changes
        valid_drones = [d for d in drone_system.drones if d.status == DroneStatus.ACTIVE]
        valid_buildings = env_system.current_city.buildings.copy()
        
        # Create ID maps for fast validation
        drone_ids = {d.id: d for d in valid_drones}
        building_ids = {b.id: b for b in valid_buildings}
        
        # Detect collisions
        collisions = self.detector.find_collisions(valid_drones, valid_buildings)
        
        # Validate collisions against current state
        valid_collisions = [
            c for c in collisions 
            if self._validate_collision(c, drone_ids, building_ids)
        ]
        
        self.resolve_collisions(valid_collisions)

    def _validate_collision(self, collision: CollisionRecord, 
                           drone_map: dict, building_map: dict) -> bool:
        """Ensure collision participants still exist"""
        if collision.collision_type == 'drone':
            return (collision.drone_id in drone_map 
                    and collision.other_id in drone_map)
        return (collision.drone_id in drone_map 
                and collision.building_id in building_map)

    def _get_drone_by_id(self, drone_id: int):
        """Retrieve drone by ID from drone system"""
        return next((d for d in self.engine.systems['drones'].drones if d.id == drone_id), None)

    def _get_building_by_id(self, building_id: int):
        """Retrieve building by ID from environment system"""
        city = self.engine.systems['environment'].current_city
        if not city:
            return None
        return next((b for b in city.buildings if b.id == building_id), None)
