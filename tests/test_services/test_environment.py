import json

import pytest

from src.services.environment import Environment


@pytest.fixture
def sample_city_data(tmp_path):
    data = {
        "cities": {
            "Test City": {
                "building_density_per_km2": 50,
                "avg_height_m": 30.0,
                "population_density_per_km2": 5000,
                "takeoff_landing_locations_count": 3
            }
        }
    }
    
    city_path = tmp_path / "cities-test.json"
    with city_path.open('w') as f:
        json.dump(data, f)
    return str(city_path)

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