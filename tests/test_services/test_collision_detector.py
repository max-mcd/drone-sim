import numpy as np
import pytest

from src.models.building import Building
from src.models.drone import Drone, DroneModel
from src.services.collision_detector import CollisionDetector


@pytest.fixture
def sample_drone_model():
    return DroneModel(
        max_speed=10.0,
        range=5000.0,
        dimensions={"length": 2.0, "width": 2.0, "height": 1.0},
        payload_capacity=2.0,
        battery_life=0.5
    )

@pytest.fixture
def drone1(sample_drone_model):
    return Drone(
        id=1,
        model=sample_drone_model,
        start_pos=np.array([0.0, 0.0, 10.0]),
        destination=np.array([100.0, 100.0, 50.0])
    )

@pytest.fixture
def drone2(sample_drone_model):
    return Drone(
        id=2,
        model=sample_drone_model,
        start_pos=np.array([5.0, 5.0, 10.0]),
        destination=np.array([200.0, 200.0, 50.0])
    )

@pytest.fixture
def building():
    return Building(
        x=50.0,
        y=50.0,
        height=100.0,
        width=20.0,
        length=20.0
    )

def test_drone_collision_detection(drone1, drone2):
    # Calculate expected collision distance (sum of radii)
    radius1 = max(drone1.model.dimensions['length'], 
                 drone1.model.dimensions['width']) / 2
    radius2 = max(drone2.model.dimensions['length'], 
                 drone2.model.dimensions['width']) / 2
    collision_distance = radius1 + radius2
    
    # Test when drones are far apart
    drone2.position = drone1.position + np.array([10.0, 10.0, 10.0])
    assert not CollisionDetector.check_drone_collision(drone1, drone2)
    
    # Test when drones are very close (within collision distance)
    drone2.position = drone1.position + np.array([0.5, 0.5, 0.5])
    assert CollisionDetector.check_drone_collision(drone1, drone2)
    
    # Test exactly at collision boundary
    # Place drone2 slightly closer than collision distance
    drone2.position = drone1.position + np.array([collision_distance - 0.1, 0.0, 0.0])
    assert CollisionDetector.check_drone_collision(drone1, drone2)
    
    # Test just beyond collision distance
    drone2.position = drone1.position + np.array([collision_distance + 0.1, 0.0, 0.0])
    assert not CollisionDetector.check_drone_collision(drone1, drone2)
    
    # Test different approach angles
    test_vectors = [
        np.array([1.0, 1.0, 0.0]) / np.sqrt(2),  # 45 degrees in xy plane
        np.array([1.0, 0.0, 1.0]) / np.sqrt(2),  # 45 degrees in xz plane
        np.array([0.0, 1.0, 1.0]) / np.sqrt(2),  # 45 degrees in yz plane
    ]
    
    for vector in test_vectors:
        # Test collision at boundary from different angles
        drone2.position = drone1.position + (collision_distance - 0.1) * vector
        assert CollisionDetector.check_drone_collision(drone1, drone2), \
            f"Failed to detect collision from vector {vector}"
        
        # Test no collision just beyond boundary
        drone2.position = drone1.position + (collision_distance + 0.1) * vector
        assert not CollisionDetector.check_drone_collision(drone1, drone2), \
            f"False collision detected from vector {vector}"

def test_building_collision_detection(drone1, building):
    # Test when drone is far from building
    drone1.position = np.array([100.0, 100.0, 50.0])
    assert not CollisionDetector.check_building_collision(drone1, building)
    
    # Test when drone is inside building bounds
    drone1.position = np.array([building.x, building.y, building.height/2])
    assert CollisionDetector.check_building_collision(drone1, building)
    
    # Calculate drone's half dimensions
    half_length = drone1.model.dimensions['length'] / 2
    half_width = drone1.model.dimensions['width'] / 2
    half_height = drone1.model.dimensions['height'] / 2
    
    # Test edge cases
    test_cases = [
        # Test x-boundary (drone approaching from right)
        (np.array([building.x + building.width/2 + half_width, building.y, 50]), True),
        (np.array([building.x + building.width/2 + half_width + 0.1, building.y, 50]), False),
        
        # Test x-boundary (drone approaching from left)
        (np.array([building.x - building.width/2 - half_width, building.y, 50]), True),
        (np.array([building.x - building.width/2 - half_width - 0.1, building.y, 50]), False),
        
        # Test y-boundary (drone approaching from front)
        (np.array([building.x, building.y + building.length/2 + half_length, 50]), True),
        (np.array([building.x, building.y + building.length/2 + half_length + 0.1, 50]), False),
        
        # Test y-boundary (drone approaching from back)
        (np.array([building.x, building.y - building.length/2 - half_length, 50]), True),
        (np.array([building.x, building.y - building.length/2 - half_length - 0.1, 50]), False),
        
        # Test z-boundary (drone approaching from top)
        (np.array([building.x, building.y, building.height + half_height]), True),
        (np.array([building.x, building.y, building.height + half_height + 0.1]), False),
        
        # Test z-boundary (drone approaching from bottom)
        (np.array([building.x, building.y, 0 - half_height]), True),
        (np.array([building.x, building.y, 0 - half_height - 0.1]), False),
    ]
    
    for position, expected_collision in test_cases:
        drone1.position = position
        assert CollisionDetector.check_building_collision(drone1, building) == expected_collision, \
            f"Failed at position {position}"

def test_building_collision_corners(drone1, building):
    # Calculate drone's half dimensions
    half_length = drone1.model.dimensions['length'] / 2
    half_width = drone1.model.dimensions['width'] / 2
    half_height = drone1.model.dimensions['height'] / 2
    
    # Test all corners of the building with drone dimensions considered
    corners = [
        (building.x + building.width/2 + half_width, 
         building.y + building.length/2 + half_length, 
         building.height + half_height),
        (building.x + building.width/2 + half_width, 
         building.y - building.length/2 - half_length, 
         building.height + half_height),
        (building.x - building.width/2 - half_width, 
         building.y + building.length/2 + half_length, 
         building.height + half_height),
        (building.x - building.width/2 - half_width, 
         building.y - building.length/2 - half_length, 
         building.height + half_height),
    ]
    
    for x, y, z in corners:
        # Test at exact corner of expanded bounds
        drone1.position = np.array([x, y, z])
        assert CollisionDetector.check_building_collision(drone1, building), \
            f"Failed at corner position ({x}, {y}, {z})"
        
        # Test just outside corner
        drone1.position = np.array([x + 0.1, y + 0.1, z + 0.1])
        assert not CollisionDetector.check_building_collision(drone1, building), \
            f"Failed at outside corner position ({x + 0.1}, {y + 0.1}, {z + 0.1})" 