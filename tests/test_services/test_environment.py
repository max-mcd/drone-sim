import csv
import tempfile
from pathlib import Path

import pytest

from src.services.environment import Environment


@pytest.fixture
def sample_city_data():
    with tempfile.NamedTemporaryFile(mode='w', delete=False, newline='') as f:
        writer = csv.DictWriter(f, fieldnames=[
            'city_name', 'building_density', 'avg_height',
            'population_density', 'takeoff_landing_locations'
        ])
        writer.writeheader()
        writer.writerow({
            'city_name': 'Test City',
            'building_density': '50',
            'avg_height': '30.0',
            'population_density': '5000',
            'takeoff_landing_locations': '3'
        })
        temp_path = f.name
    
    yield temp_path
    Path(temp_path).unlink()  # Cleanup temp file after tests

def test_environment_initialization(sample_city_data):
    dimensions = (1000.0, 1000.0, 100.0)
    env = Environment(sample_city_data, dimensions)
    
    assert env.dimensions == dimensions
    assert len(env.cities) == 1
    assert 'Test City' in env.cities
    assert env.current_city is None

def test_set_city(sample_city_data):
    env = Environment(sample_city_data, (1000.0, 1000.0, 100.0))
    
    env.set_city('Test City')
    assert env.current_city is not None
    assert env.current_city.name == 'Test City'

def test_set_invalid_city(sample_city_data):
    env = Environment(sample_city_data, (1000.0, 1000.0, 100.0))
    
    with pytest.raises(ValueError, match="City Invalid City not found"):
        env.set_city('Invalid City')

def test_building_generation(sample_city_data):
    dimensions = (1000.0, 1000.0, 100.0)
    env = Environment(sample_city_data, dimensions)
    
    city = env.cities['Test City']
    assert len(city.buildings) > 0
    
    # Test a random building's properties
    building = city.buildings[0]
    assert 0 <= building.x <= dimensions[0]
    assert 0 <= building.y <= dimensions[1]
    assert 10 <= building.width <= 30
    assert 10 <= building.length <= 30