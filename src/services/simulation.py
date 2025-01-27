from typing import List, Tuple

import numpy as np

from ..models.drone import Drone
from ..utils.config_loader import (
    load_city_data,
    load_drone_models,
    load_simulation_config,
)
from ..utils.report_generator import ReportGenerator
from .collision_detector import CollisionDetector
from .environment import Environment


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
    def __init__(self, config_path: str, drone_data_path: str, city_data_path: str):
        self.config = load_simulation_config(config_path)
        self.drone_models = load_drone_models(drone_data_path)
        
        # Load city data first
        self.city_data = load_city_data(city_data_path, self.config['simulation']['city'])
        
        # Initialize environment with simulation dimensions and city data path
        self.environment = Environment(
            city_data_path,  # Pass the path, not the data
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
        for drone_config in self.config['drones']:
            model = self.drone_models[drone_config['model']]
            start_pos = np.array([
                drone_config['start']['x'],
                drone_config['start']['y'],
                drone_config['start']['z']
            ])
            destination = np.array([
                drone_config['destination']['x'],
                drone_config['destination']['y'],
                drone_config['destination']['z']
            ])
            self.drones.append(Drone(len(self.drones), model, start_pos, destination))

    def run(self) -> None:
        """
        Run the simulation for the configured duration.
        
        Executes the simulation loop, updating drone positions and checking for collisions
        at each time step. The simulation continues until either:
        - The configured duration is reached
        - All drones have completed their routes
        
        During each time step:
        1. Updates position and state of each active drone
        2. Checks for collisions between drones and with buildings
        3. Records any detected collisions with timestamp
        4. Advances simulation time by the configured time step
        """
        dt = self.config['simulation']['time_step']
        duration = self.config['simulation']['duration']

        while self.time < duration:
            for drone in self.drones:
                if not drone.update(dt):
                    continue

            self._check_all_collisions()
            self.time += dt

    def _check_all_collisions(self) -> None:
        """
        Check for all possible collisions in the current simulation state.
        
        Performs two types of collision checks:
        1. Drone-to-drone collisions between all pairs of drones
        2. Drone-to-building collisions between each drone and all buildings
        
        When collisions are detected, they are recorded in the simulation's collision lists:
        - Drone collisions: (drone1_id, drone2_id, time)
        - Building collisions: (drone_id, building_id, time, x, y, z)
        """
        for i in range(len(self.drones)):
            for j in range(i + 1, len(self.drones)):
                if CollisionDetector.check_drone_collision(self.drones[i], self.drones[j]):
                    self.drone_collisions.append((i, j, self.time))

            for j, building in enumerate(self.environment.current_city.buildings):
                if CollisionDetector.check_building_collision(self.drones[i], building):
                    pos = self.drones[i].position
                    self.building_collisions.append((i, j, self.time, pos[0], pos[1], pos[2]))

    def _count_successful_flights(self) -> int:
        return sum(1 for drone in self.drones if drone.successful)

    def _calculate_avg_travel_time(self) -> float:
        successful_times = [drone.travel_time for drone in self.drones if drone.successful]
        return sum(successful_times) / len(successful_times) if successful_times else 0.0

    def generate_report(self) -> str:
        """Generate a formatted simulation report"""
        return ReportGenerator.generate_report(
            city_name=self.environment.current_city.name,
            simulation_time=self.time,
            total_flights=len(self.drones),
            successful_flights=self._count_successful_flights(),
            avg_travel_time=self._calculate_avg_travel_time(),
            drone_collisions=self.drone_collisions,
            building_collisions=self.building_collisions
        )