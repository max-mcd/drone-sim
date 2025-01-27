import json

import numpy as np
import pytest

from src.models.building import Building
from src.services.simulation import SimulationEngine


@pytest.fixture
def simulation_config(tmp_path):
    config = {
        'simulation': {
            'city': "Metroville",
            'time_step': 0.1,
            'duration': 3600.0,
            'dimensions': {
                'x': 2000.0,
                'y': 2000.0,
                'z': 800.0
            }
        },
        'drones': [
            {
                'model': 'Falcon-X',
                'start': [0.0, 0.0, 10.0],
                'destination': [100.0, 100.0, 50.0]
            },
            {
                'model': 'Falcon-X',
                'start': [200.0, 200.0, 10.0],
                'destination': [0.0, 0.0, 50.0]
            }
        ]
    }
    
    config_path = tmp_path / "config.json"
    json.dump(config, config_path.open('w'))
    return str(config_path)

@pytest.fixture
def drone_models(tmp_path):
    models = {
        'drone_models': {
            'Falcon-X': {
                'max_speed_m_s': 20,
                'range_km': 50,
                'dimensions_m': [1.2, 0.8, 0.5],
                'payload_capacity_kg': 5,
                'battery_life_hours': 2.5
            },
            'HawkEye': {
                'max_speed_m_s': 25,
                'range_km': 70,
                'dimensions_m': [1.5, 1.0, 0.6],
                'payload_capacity_kg': 8,
                'battery_life_hours': 2.0
            }
        }
    }
    
    models_path = tmp_path / "models.json"
    json.dump(models, models_path.open('w'))
    return str(models_path)

@pytest.fixture
def city_data(tmp_path):
    data = {
        "cities": {
            "Metroville": {
                "building_density_per_km2": 50,
                "avg_height_m": 30.0,
                "population_density_per_km2": 5000,
                "takeoff_landing_locations_count": 3
            }
        }
    }
    
    city_path = tmp_path / "cities-data.json"
    json.dump(data, city_path.open('w'))
    return str(city_path)

@pytest.fixture
def simulation(simulation_config, drone_models, city_data):
    sim = SimulationEngine(simulation_config, drone_models, city_data)
    sim.initialize_simulation()
    return sim

def test_simulation_initialization(simulation):
    assert len(simulation.drones) == 2
    assert simulation.time == 0.0
    assert len(simulation.drone_collisions) == 0
    assert len(simulation.building_collisions) == 0
    assert simulation.environment.current_city is not None
    assert simulation.environment.current_city.name == "Metroville"

def test_drone_initialization(simulation):
    assert len(simulation.drones) == 2
    drone1, drone2 = simulation.drones
    
    # Check first drone
    np.testing.assert_array_equal(drone1.position, np.array([0.0, 0.0, 10.0]))
    np.testing.assert_array_equal(drone1.destination, np.array([100.0, 100.0, 50.0]))
    
    # Check second drone
    np.testing.assert_array_equal(drone2.position, np.array([200.0, 200.0, 10.0]))
    np.testing.assert_array_equal(drone2.destination, np.array([0.0, 0.0, 50.0]))

def test_simulation_run_no_collisions(simulation):
    # Move drones far apart to avoid collisions
    simulation.drones[1].position = np.array([500.0, 500.0, 100.0])
    
    simulation.run()
    
    assert simulation.time > 0
    assert len(simulation.drone_collisions) == 0
    assert len(simulation.building_collisions) == 0

def test_simulation_run_with_drone_collision(simulation):
    # Position drones extremely close to each other to force immediate collision
    simulation.drones[1].position = simulation.drones[0].position + np.array([0.01, 0.01, 0.01])
    
    # Ensure drones don't move for this test
    simulation.drones[0].velocity = np.zeros(3)
    simulation.drones[1].velocity = np.zeros(3)
    
    # Set a very small time step to ensure collision is detected
    simulation.config['simulation']['time_step'] = 0.01
    simulation.config['simulation']['duration'] = 0.1
    
    simulation.run()
    
    assert len(simulation.drone_collisions) > 0, "No collision detected despite drones being 0.01 units apart"
    collision = simulation.drone_collisions[0]
    assert collision[0] == 0  # First drone ID
    assert collision[1] == 1  # Second drone ID
    assert collision[2] >= 0  # Collision time

def test_simulation_report(simulation):
    simulation.run()
    report = simulation.generate_report()
    
    assert isinstance(report, dict)
    assert 'drone_collisions' in report
    assert 'building_collisions' in report
    assert 'simulation_time' in report
    assert 'city' in report
    assert report['city'] == "Metroville"

def test_simulation_with_building_collision(simulation):
    # Create a building at a known position
    test_building = Building(
        x=50.0,
        y=50.0,
        height=100.0,
        width=20.0,
        length=20.0
    )
    
    # Add the test building to the current city's buildings
    simulation.environment.current_city.buildings.append(test_building)
    
    # Position drone inside the building's bounds
    simulation.drones[0].position = np.array([50.0, 50.0, 30.0])
    
    simulation.run()
    
    # Check if building collision was detected
    assert len(simulation.building_collisions) > 0
    collision = simulation.building_collisions[0]
    assert collision[0] == 0  # First drone ID
    assert collision[2] >= 0  # Collision time
