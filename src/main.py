import argparse
import logging
import threading
from pathlib import Path

import matplotlib.pyplot as plt

from .services.simulation import SimulationEngine


def setup_logging(debug_mode=False):
    """Configure logging levels for all modules"""
    # Set default level for all loggers
    logging.getLogger().setLevel(logging.WARNING)
    
    # Configure specific loggers
    loggers = {
        'src.models.drone': logging.WARNING,
        'src.services.simulation': logging.WARNING,
        'src.visualization.matplotlib_visualizer': logging.WARNING,
        'matplotlib': logging.ERROR,
        'matplotlib.font_manager': logging.ERROR,
        'PIL.PngImagePlugin': logging.ERROR
    }
    
    if debug_mode:
        # Override with DEBUG level if debug mode is enabled
        debug_loggers = ['src.models.drone', 'src.services.simulation', 'src.visualization.matplotlib_visualizer']
        for logger_name in debug_loggers:
            loggers[logger_name] = logging.DEBUG
    
    # Apply configuration
    for logger_name, level in loggers.items():
        logging.getLogger(logger_name).setLevel(level)


def main(config_path: str, real_time: bool = True):
    setup_logging(debug_mode=False)
    config_path = Path.cwd() / config_path
    
    # Create engine with visualization setup
    engine = SimulationEngine(
        str(config_path),
        str(Path.cwd() / "data/drones/drone_models.json"),
        str(Path.cwd() / "data/cities/cities-data.json"),
        real_time=real_time
    )
    engine.initialize_simulation()
    
    try:
        print(f"Running simulation in {'real-time' if real_time else 'fast'} mode...")
        
        # Show visualization window first
        plt.show(block=False)  # Non-blocking show to setup window
        
        if not real_time:
            sim_thread = threading.Thread(target=engine.run)
            sim_thread.start()
        else:
            engine.run()
            
        # Now block until window is closed
        plt.show(block=True)
        
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user")
    finally:
        plt.close('all')


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("config", help="Path to simulation config file")
    parser.add_argument("--fast", action="store_true", help="Run in fast mode without visualization")
    args = parser.parse_args()
    
    main(args.config, real_time=not args.fast)