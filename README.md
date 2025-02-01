# DRONE SIMULATION PROJECT

This project is a simulation of a drone flight safety system. It is designed to 
simulate the flight of a drone in a 3D environment and to test the flight safety system.

## Documentation

## Project Structure

The project is structured as follows:

- `src/`: Source code for the simulation system.
- `tests/`: Unit tests for the simulation system.
- `data/`: Data for the simulation system.
- `output/`: Contains a screenshot of visualization once the simulation is complete.
- `docs/`: Documentation for the simulation system. (Not yet implemented)
- `pyproject.toml`: Project configuration file.
- `README.md`: This file.

## Run the simulation

You can run the simulation in several ways:

- using the `run.sh` script: uses the default configuration file (`data/config/simulation_config.json`)
- using the `python -m src.main data/config/simulation_config.json` command, where you can specify 
a different configuration file

### Example configurations

Set your simulation configuration before running the simulation.
The `data/config` directory contains an example configuration file.

### From repository root (drone-simulation)

```bash
./drone-sim/scripts/run.sh
```

```bash
python -m drone-sim.src.main drone-sim/data/config/simulation_config.json
```

## From module directory (drone-sim)

```bash
./scripts/run.sh
```

```bash
python -m src.main data/config/simulation_config.json
```

### Command-line options

- `--fast`: Run simulation without real-time delays
- `--debug`: Enable detailed debug logging
- `--avoid-collisions`: Enable the collision avoidance system
- The first argument is the path to a simulation configuration file

From `drone-sim` directory, you can run the simulation in debug mode and redirect 
the output to a file:

```bash
./scripts/run.sh --debug  &> simulation_run_result.txt
```

Note: The output file will be created in the `drone-sim` directory and is in the .gitignore file.

## Collision Avoidance System

The simulation includes a collision avoidance system that can be enabled with the `--avoid-collisions` flag.
When enabled, the system:

1. Predicts potential collisions up to 5 seconds ahead
2. Uses a hierarchical decision system to determine which drone should yield
3. Applies speed adjustments to avoiding drones based on:
   - Distance to potential collision
   - Required separation between drones
   - Relative velocities

The system uses different safety buffers:

- Large buffer (30-40m) for initial collision prediction
- Medium buffer (20-25m) for non-yielding drones
- Small buffer (10-15m) for drones already in avoidance mode

This allows drones to maintain safe distances while still being able to pass each other
once avoidance maneuvers are initiated.

## Technical Details

### Components and their responsibilities

```text
Drone Flight Simulation System
│
├── Core Components
│   ├── SimulationEngine (simulation.py)
│   │   └── Main orchestrator that:
│   │       - Loads configuration
│   │       - Manages simulation loop
│   │       - Coordinates drones and environment
│   │       - Handles collision detection
│   │
│   ├── Environment (environment.py)
│   │   └── Manages city layout:
│   │       - Generates buildings
│   │       - Handles city configuration
│   │       - Maintains spatial data
│   │
│   └── StateManager (state_manager.py)
│       └── Handles state propagation:
│           - Maintains current simulation state
│           - Notifies observers of changes
│           - Coordinates visualization updates
│
├── Models
│   ├── Drone
│   │   ├── Properties: position, velocity, status
│   │   ├── Behaviors: movement, waypoint navigation
│   │   └── Uses FlightPath for navigation
│   │
│   ├── FlightPath
│   │   ├── Manages waypoint sequences
│   │   └── Handles waypoint progression
│   │
│   ├── Building
│   │   └── Represents obstacles with dimensions
│   │
│   └── SimulationState
│       └── Captures complete system state
│
├── Visualization
│   └── MatplotlibVisualizer
│       ├── Real-time display of:
│       │   - Drone positions and trails
│       │   - Building layouts
│       │   - Flight paths
│       │   - Collision indicators
│       └── Handles animation and updates
│
└── Utilities
    ├── ConfigLoader
    │   └── Handles JSON configuration parsing
    │
    └── ReportGenerator
        └── Generates simulation statistics
```

### Main Data Flow

```text
Configuration Files ─────┐
                        ▼
User Input ────► SimulationEngine ◄────► Environment
                    │   ▲                    │
                    │   │                    │
                    ▼   │                    ▼
                StateManager             Buildings
                    │   ▲
                    │   │
                    ▼   │
                MatplotlibVisualizer
                    │
                    ▼
                User Display
```

### Simulation Loop  

```text
Initialize ──► Load Config ───┬──► Setup Environment ──► Generate Buildings
       │                      │
       │                      └──► Create Drones
       │
       ▼
  Start Simulation Loop
       │
       ▼
  Update Drone Positions ◄─────┐
       │                       │
       ▼                       │
  Check Collisions             │
       │                       │
       ▼                       │
  Update State                 │
       │                       │
       ▼                       │
  Notify Visualizer            │         Exit Conditions:
       │                       │         - All drones complete
       ├───────────────────────┘         - Duration reached
       │                                 - User interrupts
       ▼
  Check Exit Conditions
       │
       ▼
  Generate Final Report
       │
       ▼
  Save Visualization
```
