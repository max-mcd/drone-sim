import argparse
import logging
import threading
from pathlib import Path

import matplotlib.pyplot as plt

from .services.simulation import SimulationEngine


def setup_logging(debug_mode=False):
    """Configure logging levels for all modules"""
    logging.basicConfig(
        level=logging.DEBUG if debug_mode else logging.WARNING,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        datefmt='%H:%M:%S'
    )
    
    # Configure specific loggers
    loggers = {
        'src.models.drone': logging.WARNING,
        'src.models.drone_model': logging.WARNING,
        'src.services.simulation': logging.WARNING,
        'src.visualization.matplotlib_visualizer': logging.WARNING,
        'src.services.collision_detector': logging.WARNING,
        'src.services.collision_avoidance': logging.WARNING,
        'matplotlib': logging.ERROR,
        'PIL.PngImagePlugin': logging.ERROR
    }
    
    if debug_mode:
        debug_loggers = [
            'src.models.drone',
            'src.models.drone_model',
            'src.services.simulation',
            'src.visualization.matplotlib_visualizer',
            'src.services.state_manager',
            'src.services.collision_detector',
            'src.services.collision_avoidance',
            'src.utils.config_loader'
        ]
        for logger_name in debug_loggers:
            loggers[logger_name] = logging.DEBUG
    
    # Apply configuration
    for logger_name, level in loggers.items():
        logging.getLogger(logger_name).setLevel(level)


def main(config_path: str, real_time: bool = True):
    # Handle both running from repo root and module directory
    base_path = Path(__file__).parent.parent  # Gets to drone-sim/src parent (drone-sim directory)
    config_path = Path(config_path)
    
    # If running from repo root (path starts with drone-sim/)
    if str(config_path).startswith('drone-sim/'):
        config_path = config_path.relative_to('drone-sim')
    
    # Resolve paths relative to base_path
    config_path = base_path / config_path
    drone_models_path = base_path / "data/drones/drone_models.json"
    cities_data_path = base_path / "data/cities/cities-data.json"

    parser = argparse.ArgumentParser()
    parser.add_argument("config", help="Path to simulation config file")
    parser.add_argument("--fast", action="store_true", help="Run in fast mode without visualization")
    parser.add_argument("--debug", action="store_true", help="Enable debug logging")
    parser.add_argument("--avoid-collisions", action="store_true", 
                       help="Enable collision avoidance system")
    args = parser.parse_args()
    
    setup_logging(debug_mode=args.debug)
    
    # Create engine with correct paths
    engine = SimulationEngine(
        str(config_path),
        str(drone_models_path),
        str(cities_data_path),
        real_time=real_time,
        avoid_collisions=args.avoid_collisions
    )
    engine.initialize_simulation()
    
    # Show the figure
    plt.ion()
    plt.show()
    plt.pause(0.5)  # Give window time to initialize

    try:
        print(f"Running simulation in {'real-time' if real_time else 'fast'} mode...")
        
        # Always run simulation in background thread
        sim_thread = threading.Thread(target=engine.run)
        sim_thread.start()

        # Main thread handles visualization updates
        while sim_thread.is_alive():
            plt.pause(0.01)  # Process matplotlib events

        plt.ioff()
        plt.show(block=True)  # This blocks until window is closed
            
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user")
    finally:
        plt.close('all')


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("config", help="Path to simulation config file")
    parser.add_argument("--fast", action="store_true", help="Run in fast mode without visualization")
    parser.add_argument("--debug", action="store_true", help="Enable debug logging")
    parser.add_argument("--avoid-collisions", action="store_true", 
                       help="Enable collision avoidance system")
    args = parser.parse_args()
    
    setup_logging(debug_mode=args.debug)
    main(args.config, real_time=not args.fast)