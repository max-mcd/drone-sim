import argparse
from pathlib import Path

import matplotlib.pyplot as plt

from .services.simulation import SimulationEngine


def main(config_path: str, real_time: bool = True):
    # TODO: Validate configs before running simulation
    
    # Convert relative path to absolute path
    config_path = Path.cwd() / config_path
    
    # Debug prints
    print(f"Config path: {config_path}")
    print(f"Working directory: {Path.cwd()}")
    
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
            plt.show()  # Only show interactive window in real-time mode
        
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