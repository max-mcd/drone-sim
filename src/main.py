import argparse
import logging
from pathlib import Path

import matplotlib.pyplot as plt

from .services.simulation import SimulationEngine

def setup_logging(debug: bool = True):
    """Configure logging for the entire application"""
    log_level = logging.DEBUG if debug else logging.INFO
    
    # Configure the root logger
    logging.basicConfig(
        level=log_level,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        datefmt='%H:%M:%S'
    )
    
    # Suppress matplotlib and PIL debug logs regardless of debug setting
    logging.getLogger('matplotlib').setLevel(logging.WARNING)
    logging.getLogger('PIL').setLevel(logging.WARNING)

def main(config_path: str, real_time: bool = True):
    # Setup logging first
    setup_logging(debug=True)  # Set to True to see all debug messages
    
    # Convert relative path to absolute path
    config_path = Path.cwd() / config_path
    
    # Debug prints
    logger = logging.getLogger(__name__)
    logger.debug(f"Config path: {config_path}")
    logger.debug(f"Working directory: {Path.cwd()}")
    
    engine = SimulationEngine(
        str(config_path),
        str(Path.cwd() / "data/drones/drone_models.json"),
        str(Path.cwd() / "data/cities/cities-data.json"),
        real_time=real_time  # Pass real_time flag
    )

    engine.initialize_simulation()
    
    try:
        print(f"Running simulation in {'real-time' if real_time else 'fast'} mode...")
        engine.run()
        
        if real_time:
            print("\nSimulation complete. Close the visualization window to exit.")
            plt.show()  # This will block until user closes the window
        else:
            print("\nSimulation complete. Visualization will remain open for 30 seconds...")
            plt.show(block=False)
            plt.pause(30)  # Keep window open longer to see final state
        
        print(engine.generate_report())
        
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user")
    finally:
        plt.close('all')  # Clean up matplotlib windows

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("config", help="Path to simulation config file")
    parser.add_argument("--fast", action="store_true", help="Run in fast mode without visualization")
    args = parser.parse_args()
    
    main(args.config, real_time=not args.fast)