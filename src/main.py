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
        'src.services.simulation': logging.WARNING,
        'src.visualization.matplotlib_visualizer': logging.WARNING,
        'matplotlib': logging.ERROR,
        'PIL.PngImagePlugin': logging.ERROR
    }
    
    if debug_mode:
        debug_loggers = [
            'src.models.drone',
            'src.services.simulation',
            'src.visualization.matplotlib_visualizer',
            'src.services.state_manager',
            'src.utils.config_loader'
        ]
        for logger_name in debug_loggers:
            loggers[logger_name] = logging.DEBUG
    
    # Apply configuration
    for logger_name, level in loggers.items():
        logging.getLogger(logger_name).setLevel(level)


def main(config_path: str, real_time: bool = True):
    config_path = Path.cwd() / config_path
    
    # Create engine
    engine = SimulationEngine(
        str(config_path),
        str(Path.cwd() / "data/drones/drone_models.json"),
        str(Path.cwd() / "data/cities/cities-data.json"),
        real_time=real_time
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

        # Show final state
        plt.ioff()
        plt.show(block=True)
        
        # Show report after window is closed
        report = engine.generate_report()
        print("\n" + "="*50)
        print("Simulation Report:")
        print("="*50)
        print(report)
            
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user")
    finally:
        plt.close('all')


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("config", help="Path to simulation config file")
    parser.add_argument("--fast", action="store_true", help="Run in fast mode without visualization")
    parser.add_argument("--debug", action="store_true", help="Enable debug logging")
    args = parser.parse_args()
    
    setup_logging(debug_mode=args.debug)
    main(args.config, real_time=not args.fast)