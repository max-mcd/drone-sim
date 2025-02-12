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
├── Core Framework
│   ├── Engine (engine.py) - Orchestrates system updates and main loop
│   ├── Event Bus (events.py) - Pub/sub system for cross-component communication
│   ├── State Manager (state.py) - Maintains current simulation state
│   └── Configuration Loader (config_loader.py) - Loads and validates configs
│
├── Component Systems
│   ├── Collision System (collision/)
│   │   ├── Detector (detector.py) - Checks collisions between Collidable objects
│   │   ├── Resolver (resolver.py) - Applies avoidance strategies
│   │   └── Strategies (strategies/) - Different collision resolution implementations
│   │       ├── emergency.py - Priority-based emergency avoidance
│   │       └── no_op.py - No avoidance (baseline)
│   │
│   ├── Pathfinding System (pathfinding/) - Calculates optimal routes
│   ├── Environment System (environment.py) - Manages buildings/terrain
│   └── Visualization System (visualization/) - Renders simulation state
│       └── matplotlib.py - Matplotlib-based visualizer
│
├── Domain Models
│   ├── Drone (drone.py) - Core drone logic and state
│   ├── Building (building.py) - Static obstacle representation
│   └── Flight Path (flight_path.py) - Waypoint sequence management
│
├── Protocols (protocols.py) - Defines interface contracts (Collidable, Renderable)
│
└── Utilities
    ├── Config Validation (config_validator.py) - Schema-based validation
    └── Reporting (reporting.py) - Generates simulation statistics
```

### Main Data Flow

#### 1. Initialization Phase

```text
[config/simulation.yaml] → ConfigLoader → Validate → [SimulationEngine]
     │
     └──► [models/drone.py] Create drones
     └──► [models/building.py] Generate environment
     └──► [systems/__init__.py] Initialize systems
```

#### 2. Simulation Run Phase (per time step)

```text
Time Step Trigger
     │
     ▼
[core/engine.py] → Update Systems:
     │
     ├──► [systems/environment/system.py] Update weather/terrain
     │
     ├──► [systems/pathfinding/system.py] Recalculate routes
     │       │
     │       └──► [models/flight_path.py] Adjust waypoints
     │
     ├──► [systems/collision/detector.py] Check Collidable objects
     │       │
     │       ├──► [protocols.py] Verify Collidable compliance
     │       │
     │       └──► [systems/collision/resolver.py] Apply strategy:
     │               ├──► [strategies/emergency.py] Priority avoidance
     │               └──► [strategies/no_op.py] Default passthrough
     │
     ├──► [core/state.py] Record collision events/position updates
     │       │
     │       └──► [utils/reporting.py] Log metrics
     │
     └──► [systems/visualization/matplotlib.py] Render frame
             │
             └──► [protocols.py] Use Renderable.get_visual_state()

```

#### 3. Reporting Phase

```text
[core/state.py] Simulation History
     │
     ▼
[utils/reporting.py] → Generate:
     ├──► Collision Report
     ├──► Performance Metrics
     └──► Flight Path Analysis
```

### Simulation Loop  

```text
Initialize ──► Load Config (config_loader.py) ──┬──► Setup Environment (environment.py) ──► Generate Buildings (models/building.py)
       │                                        │
       │                                        └──► Create Drones (models/drone.py) with FlightPaths (models/flight_path.py)
       │
       ▼
  Start Simulation Loop (core/engine.py)
       │
       ▼
  Update Systems:
       ├──► Drone Physics (models/drone.py update())
       ├──► Pathfinding (pathfinding.py)
       └──► Collision System:
               │
               ├──► Broad Phase Check (detector.py)  # Quick spatial partitioning check using grid-based grouping
               ├──► Narrow Phase Check (detector.py)  # Precise geometric collision check with position prediction
               ├──► Strategy Resolution (resolver.py):
               │       ├──► Hierarchical (strategies/hierarchical.py)
               │       ├──► Emergency (strategies/emergency.py)
               │       └──► No-Op (strategies/no_op.py)
               │
               └──► Apply Avoidance (avoidance_system.py)
       │
       ▼
  Update State (state.py) ───► Log Collisions (reporting.py)
       │
       ▼
  Visualize (matplotlib.py) ◄─── Renderable Protocol (protocols.py)
       │
       ▼
  Check Exit Conditions:
       ├──► All drones reached destinations (flight_path.py)
       ├──► Max duration reached
       └──► User interrupt
       │
       ▼
  Generate Report (reporting.py) ──► Collision Stats ◄─── Protocols (Collidable)
       │
       ▼
  Persist Results ───┬──► Visualization Frames
                     └──► Simulation Metrics
```
