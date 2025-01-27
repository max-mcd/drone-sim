from pathlib import Path

from .services.simulation import SimulationEngine


def main(config_path: str):
    # TODO: Validate configs before running simulation
    
    # Convert relative path to absolute path
    config_path = Path.cwd() / config_path
    
    # Debug prints
    print(f"Config path: {config_path}")
    print(f"Working directory: {Path.cwd()}")
    
    engine = SimulationEngine(
        str(config_path),
        str(Path.cwd() / "data/drones/drone_models.json"),
        str(Path.cwd() / "data/cities/cities-data.json")
    )

    engine.initialize_simulation()
    engine.run()
    print(engine.generate_report())

if __name__ == "__main__":
    import sys
    main(sys.argv[1])