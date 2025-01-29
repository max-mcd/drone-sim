import logging
import sys
import time
from typing import List, Tuple
from pathlib import Path

import numpy as np

from ..models.drone import Drone
from ..models.flight_path import FlightPath
from ..models.simulation_state import SimulationState
from ..services.state_manager import SimulationStateManager
from ..utils.config_loader import (
    load_city_data,
    load_drone_models,
    load_simulation_config,
)
from ..utils.report_generator import ReportGenerator
from ..visualization.matplotlib_visualizer import MatplotlibVisualizer
from .collision_detector import CollisionDetector
from .environment import Environment

logger = logging.getLogger(__name__)


class SimulationEngine:
    """
    A core simulation engine that manages and updates the state of multiple drones in a city environment.
    
    This class is responsible for:
    - Loading and initializing simulation configuration, drone models, and city environment
    - Managing the lifecycle of multiple drone agents
    - Detecting and recording collisions between drones and buildings
    - Advancing the simulation state based on configured time steps
    - Tracking simulation time and duration
    
    The simulation runs until either the configured duration is reached or all drones have
    completed their routes. During each time step, drone positions are updated and checked
    for potential collisions with other drones or buildings in the environment.
    """
    def __init__(self, config_path: str, drone_data_path: str, city_data_path: str, real_time: bool = True):
        logger.info(f"Initializing simulation in {'real-time' if real_time else 'fast'} mode")
        self.config = load_simulation_config(config_path)
        logger.debug(f"Loaded config with {len(self.config['drones'])} drones")
        self.drone_models = load_drone_models(drone_data_path)
        self.real_time = real_time  # Store real_time mode
        
        # Load city data first
        self.city_data = load_city_data(city_data_path, self.config['simulation']['city'])
        
        # Initialize environment with simulation dimensions and city data path
        self.environment = Environment(
            city_data_path,
            (
                self.config['simulation']['dimensions']['x'],
                self.config['simulation']['dimensions']['y'],
                self.config['simulation']['dimensions']['z']
            )
        )
        # Initialize city using config
        self.environment.set_city(self.config['simulation']['city'])
        
        self.drones: List[Drone] = []
        self.drone_collisions: List[Tuple[int, int, float]] = []
        self.building_collisions: List[Tuple[int, int, float, float, float, float]] = []
        self.time = 0.0
        
        self.state_manager = SimulationStateManager()
        
        # Create and register visualizer - pass real_time mode
        dimensions = (
            self.config['simulation']['dimensions']['x'],
            self.config['simulation']['dimensions']['y'],
            self.config['simulation']['dimensions']['z']
        )
        self.visualizer = MatplotlibVisualizer(dimensions, real_time=self.real_time)
        self.state_manager.add_observer(self.visualizer.on_state_update)
        
        # Add simulation state flag
        self.simulation_complete = False

    def initialize_simulation(self) -> None:
        # City is already initialized in __init__
        self._initialize_drones()

    def _initialize_drones(self) -> None:
        """
        Initialize drone objects based on configuration settings.
        
        Creates Drone instances for each drone configuration entry, setting up:
        - Drone model based on the specified model name from loaded drone models
        - Starting position coordinates (x, y, z)
        - Destination coordinates (x, y, z)
        
        Each drone is assigned an incremental ID based on its position in the drones list.
        The initialized drones are stored in the simulation engine's drones list.
        """
        logger.debug("Initializing drones")
        for i, drone_config in enumerate(self.config['drones']):
            model = self.drone_models[drone_config['model']]
            flight_path = FlightPath([
                np.array(waypoint) for waypoint in drone_config['waypoints']
            ])
            logger.debug(f"Initializing drone {i} with {len(flight_path.waypoints)} waypoints")
            self.drones.append(Drone(len(self.drones), model, flight_path))

    def _update_state(self) -> None:
        """Create and update current simulation state"""
        logger.debug(f"""
            Creating new state:
            Time: {self.time}
            Drones: {len(self.drones)}
            Active drones: {sum(1 for d in self.drones if d.status == 'active')}
        """)
        
        new_state = SimulationState(
            time=self.time,
            drones=self.drones,
            buildings=self.environment.current_city.buildings,
            drone_collisions=self.drone_collisions,
            building_collisions=self.building_collisions
        )
        
        logger.debug("State created, updating observers")
        self.state_manager.update_state(new_state)

    def run(self) -> None:
        """
        Run the simulation for the configured duration.
        
        Executes the simulation loop, updating drone positions and checking for collisions
        at each time step. The simulation continues until either:
        - The configured duration is reached
        - All drones have completed their routes or collided
        
        During each time step:
        1. Updates position and state of each active drone
        2. Checks for collisions between drones and with buildings
        3. Records any detected collisions with timestamp
        4. Advances simulation time by the configured time step
        """
        logger.debug(f"""
            Starting simulation:
            Time step: {self.config['simulation']['time_step']}
            Duration: {self.config['simulation']['duration']}
            Drones: {len(self.drones)}
            Real-time mode: {self.real_time}
        """)
        logger.debug("Starting simulation run")
        dt = self.config['simulation']['time_step']
        duration = self.config['simulation']['duration']
        
        # Log initial setup
        for drone in self.drones:
            logger.info(f"""
                Initial drone {drone.id} setup:
                Speed: {drone.model.max_speed} m/s
                Distance to travel: {np.linalg.norm(drone.destination - drone.start_pos):.1f}m
                Expected travel time: {np.linalg.norm(drone.destination - drone.start_pos)/drone.model.max_speed:.1f}s
            """)
        
        # Run simulation
        try:
            while self.time < duration and not self.simulation_complete:
                # Update drones
                for drone in self.drones:
                    drone.update(dt)
                
                # Check collisions
                self._check_all_collisions()
                
                # Create and push new state
                self._update_state()
                
                # Check if all drones are done
                if all(drone.status in ['successful', 'collided'] for drone in self.drones):
                    logger.info("All drones have completed their routes")
                    self.simulation_complete = True
                    self._update_state()  # Final state update
                    
                    # Generate and save report
                    report = self.generate_report()
                    
                    # Save report to file
                    output_dir = Path.cwd() / "output"
                    output_dir.mkdir(exist_ok=True)
                    report_path = output_dir / f"simulation_report_{self.time:.1f}s.txt"
                    with open(report_path, 'w') as f:
                        f.write("="*50 + "\n")
                        f.write("Simulation Report\n")
                        f.write("="*50 + "\n")
                        f.write(report)
                    
                    # Output to console
                    sys.stdout.write("\n" + "="*50 + "\n")
                    sys.stdout.write("Simulation Report:\n")
                    sys.stdout.write("="*50 + "\n")
                    sys.stdout.write(report + "\n")
                    sys.stdout.write(f"\nReport saved to: {report_path}\n")
                    sys.stdout.flush()
                    
                    break
                
                self.time += dt
                
                if self.real_time:
                    time.sleep(dt)
        except KeyboardInterrupt:
            logger.info("Simulation interrupted by user")
        except Exception as e:
            logger.error(f"Error during simulation: {e}", exc_info=True)

    def _check_all_collisions(self) -> None:
        """
        Check for all possible collisions in the current simulation state.
        
        Uses a triangular comparison pattern to efficiently check drone-to-drone collisions:
        - Drone 0 checks against: 1, 2, 3, ..., n-1
        - Drone 1 checks against: 2, 3, ..., n-1
        - Drone 2 checks against: 3, ..., n-1
        And so on...

        This approach:
        1. Avoids redundant checks (if we checked 1 vs 0, we don't need to check 0 vs 1)
        2. Breaks early when a collision is found
        3. Skips already collided drones
        
        Total comparisons in worst case (no collisions): n(n-1)/2
        where n is the number of drones.

        The method performs two types of collision checks:
        1. Drone-to-drone collisions between all pairs of drones
        2. Drone-to-building collisions between each drone and all buildings
        
        When collisions are detected:
        - Drones are marked as 'collided' and stop moving
        - Collisions are recorded with timestamp and coordinates
        - Drone collisions: (drone1_id, drone2_id, time)
        - Building collisions: (drone_id, building_id, time, x, y, z)
        """
        for i in range(len(self.drones)):
            drone = self.drones[i]
            old_status = drone.status
            
            # Skip if drone has already collided
            if drone.status == 'collided':
                continue
                
            for j in range(i + 1, len(self.drones)):
                # Skip if other drone has already collided
                if self.drones[j].status == 'collided':
                    continue
                    
                if CollisionDetector.check_drone_collision(drone, self.drones[j]):
                    self.drone_collisions.append((i, j, self.time))
                    # Mark both drones as collided
                    self.drones[i].status = 'collided'
                    self.drones[j].status = 'collided'
                    break  # Stop checking this drone against others

            # Only check building collisions if drone hasn't collided with another drone
            if drone.status != 'collided':
                for j, building in enumerate(self.environment.current_city.buildings):
                    if CollisionDetector.check_building_collision(drone, building):
                        pos = drone.position
                        self.building_collisions.append((i, j, self.time, pos[0], pos[1], pos[2]))
                        drone.status = 'collided'
                        break  # Stop checking this drone against other buildings

            if drone.status != old_status:
                logger.info(f"""
                    Drone {drone.id} status changed:
                    From: {old_status}
                    To: {drone.status}
                    Position: {drone.position}
                    Time: {self.time:.1f}s
                """)

    def _count_successful_flights(self) -> int:
        return sum(1 for drone in self.drones if drone.successful)

    def _calculate_avg_travel_time(self) -> float:
        successful_times = [drone.travel_time for drone in self.drones if drone.successful]
        return sum(successful_times) / len(successful_times) if successful_times else 0.0

    def generate_report(self) -> str:
        """Generate a formatted simulation report"""
        successful_times = [d.travel_time for d in self.drones if d.successful]
        avg_success_time = sum(successful_times) / len(successful_times) if successful_times else 0.0
        
        return ReportGenerator.generate_report(
            city_name=self.environment.current_city.name,
            simulation_time=self.time,
            total_flights=len(self.drones),
            successful_flights=self._count_successful_flights(),
            avg_travel_time=avg_success_time,
            drone_collisions=self.drone_collisions,
            building_collisions=self.building_collisions,
            building_count=len(self.environment.current_city.buildings)
        )