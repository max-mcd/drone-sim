import numpy as np
import pytest

from src.models.drone import Drone, DroneModel


@pytest.fixture
def sample_drone_model():
    return DroneModel(
        max_speed=10.0,
        range=5000.0,
        dimensions={"length": 0.5, "width": 0.5, "height": 0.2},
        payload_capacity=2.0,
        battery_life=0.5  # 30 minutes in hours
    )

@pytest.fixture
def sample_drone(sample_drone_model):
    start_pos = np.array([0.0, 0.0, 0.0])
    destination = np.array([100.0, 100.0, 50.0])
    return Drone(id=1, model=sample_drone_model, start_pos=start_pos, destination=destination)

def test_drone_model_initialization():
    model = DroneModel(
        max_speed=10.0,
        range=5000.0,
        dimensions={"length": 0.5, "width": 0.5, "height": 0.2},
        payload_capacity=2.0,
        battery_life=0.5
    )
    
    assert model.max_speed == 10.0
    assert model.range == 5000.0
    assert model.dimensions == {"length": 0.5, "width": 0.5, "height": 0.2}
    assert model.payload_capacity == 2.0
    assert model.battery_life == 0.5

def test_drone_initialization(sample_drone_model):
    start_pos = np.array([0.0, 0.0, 0.0])
    destination = np.array([100.0, 100.0, 50.0])
    drone = Drone(id=1, model=sample_drone_model, start_pos=start_pos, destination=destination)
    
    assert drone.id == 1
    assert drone.model == sample_drone_model
    np.testing.assert_array_equal(drone.position, start_pos)
    np.testing.assert_array_equal(drone.destination, destination)
    np.testing.assert_array_equal(drone.velocity, np.zeros(3))
    assert drone.battery_remaining == sample_drone_model.battery_life * 3600

def test_drone_update_movement(sample_drone):
    initial_position = sample_drone.position.copy()
    dt = 0.1  # 100ms timestep
    
    # Update drone position
    result = sample_drone.update(dt)
    
    assert result is True  # Drone should still be operational
    assert not np.array_equal(sample_drone.position, initial_position)  # Position should have changed
    assert np.all(sample_drone.velocity <= sample_drone.model.max_speed)  # Speed should not exceed max

def test_drone_battery_depletion(sample_drone):
    dt = 1800  # 30 minutes in seconds
    
    # Update should succeed initially
    assert sample_drone.update(dt) is True
    
    # Second update should fail due to depleted battery
    assert sample_drone.update(dt) is False
    assert sample_drone.battery_remaining <= 0

def test_drone_reaches_destination(sample_drone):
    # Place drone very close to destination
    sample_drone.position = sample_drone.destination - np.array([0.1, 0.1, 0.1])
    dt = 0.1
    
    sample_drone.update(dt)
    
    # Verify drone moves towards destination
    np.testing.assert_array_almost_equal(
        sample_drone.position,
        sample_drone.destination,
        decimal=1
    ) 