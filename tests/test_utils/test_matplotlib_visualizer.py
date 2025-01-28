from src.models.simulation_state import SimulationState
from utils.matplotlib_visualizer import MatplotlibVisualizer


def test_visualizer_initialization():
    dimensions = (1000.0, 1000.0, 500.0)
    visualizer = MatplotlibVisualizer(dimensions)
    assert visualizer.ax is not None
    
def test_state_update():
    dimensions = (1000.0, 1000.0, 500.0)
    visualizer = MatplotlibVisualizer(dimensions)
    
    state = SimulationState(
        time=0.0,
        drones=[{'position': [0.0, 0.0, 0.0], 'velocity': [0.0, 0.0, 0.0]}],
        buildings=[{'position': [100.0, 100.0], 'dimensions': [20.0, 20.0, 50.0]}],
        drone_collisions=[],
        building_collisions=[]
    )
    
    visualizer.on_state_update(state)  # Should not raise any exceptions 