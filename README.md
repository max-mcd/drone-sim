# Drone Flight Safety Simulation System

A sophisticated Python-based drone simulation system designed to test and validate flight safety protocols, collision avoidance strategies, and multi-drone coordination in urban environments.

## Features

- Real-time 3D simulation of multiple drones with different models and capabilities
- Advanced collision avoidance system with multiple strategies
- City environment simulation with buildings and terrain
- Configurable simulation parameters including time steps and real-time factors
- Visualization system using Matplotlib
- Comprehensive event logging and reporting
- Multiple collision avoidance strategies (hierarchical, probabilistic, emergency, noop)

## Installation

Requires Python 3.11 or higher.

```bash
# Clone the repository
git clone [repository-url]
cd drone-simulation-prototype

# Install dependencies
pip install .

# For development
pip install .[dev]
```

## Quick Start

1. Run the simulation with default configuration:
```bash
./scripts/run.sh
```

2. Run with custom configuration:
```bash
python -m drone_sim.main --config path/to/config.json
```

## Command-line Options

- `--config`: Path to simulation configuration file (default: config/simulation_config.json)
- `--fast`: Run simulation without real-time delays
- `--debug`: Enable detailed debug logging
- `--avoid-collisions <strategy>`: Select collision avoidance strategy (default: basic)
  - Available strategies: hierarchical, probabilistic, emergency, noop

## Configuration

### Simulation Configuration (simulation_config.json)

```json
{
    "simulation": {
        "time_step": 0.1,
        "real_time_factor": 1.0,
        "duration": 120.0,
        "dimensions": {
            "x": 2000.0,
            "y": 2000.0,
            "z": 800.0
        }
    }
}
```

### Collision Avoidance Strategies (collision_strategies.yaml)

Available strategies:
- **Hierarchical**: Priority-based avoidance using altitude levels
- **Probabilistic**: Risk assessment based avoidance
- **Emergency**: Last-resort collision prevention
- **NoOp**: Baseline strategy with no avoidance (for testing)

Configure strategy parameters in `config/collision_strategies.yaml`.

## Project Structure

```
drone-simulation-prototype/
├── config/                 # Configuration files
├── data/                   # Simulation data (cities, drone models)
├── output/                 # Simulation output and visualizations
├── schemas/               # JSON schemas for validation
├── src/
│   └── drone_sim/
│       ├── core/          # Core simulation engine and event system
│       ├── models/        # Domain models (drones, buildings, flight paths)
│       ├── systems/       # Simulation subsystems
│       │   ├── collision/     # Collision detection and avoidance strategies
│       │   ├── environment/   # City, terrain, and weather simulation
│       │   ├── movement/      # Drone movement and physics
│       │   ├── pathfinding/   # Route calculation and optimization
│       │   ├── state/         # Simulation state management
│       │   └── visualization/ # Real-time visualization
│       └── utils/         # Configuration and reporting utilities
└── tests/                 # Unit and integration tests
```

## Core Systems

### Control Authority System

The simulation implements control authority through multiple layers:

1. **Physical Authority**
   - Each drone model defines physical control limits:
     * Maximum acceleration/deceleration rates
     * Speed and altitude constraints
     * Response time parameters
   - Safety buffers automatically adjust based on current speed and maneuverability

2. **Operational Authority**
   - Priority-based decision making in collision scenarios
   - Emergency status overrides for critical situations
   - Battery life constraints affecting available actions
   - Payload capacity influencing maneuverability

3. **Decision Authority**
   - Hierarchical collision avoidance with clear authority chains
   - Dynamic authority transfer based on drone status
   - Automatic safety protocol engagement
   - Real-time adjustment of control parameters

This multi-layered approach ensures safe and efficient drone operations while respecting physical limitations and operational priorities.

### Collision Avoidance System

The collision avoidance system uses a multi-layered approach:

1. **Detection**
   - Dynamic collision radius calculation based on drone speed and safety parameters
   - Direct distance-based collision checks between all object pairs
   - Note: Current implementation needs improvement to handle spatial partitioning efficiently

2. **Resolution Strategies**
   - Hierarchical: Altitude-based priority system
   - Probabilistic: Risk-assessment based decisions
   - Emergency: Last-resort collision prevention
   - NoOp: Baseline strategy with no avoidance (for testing)

3. **Safety Buffers**
   - Detection buffer: 30-40m for initial detection
   - Avoidance buffer: 20-25m for active avoidance
   - Emergency buffer: 10-15m minimum separation

### Environment System

- Loads and manages city data from JSON files
- Handles terrain and building collision detection
- Supports multiple drone models with different characteristics

### Visualization System

- Real-time 3D visualization using Matplotlib
- Displays drone positions, paths, and collision predictions
- Configurable view options and debug overlays

## Development

### Running Tests

```bash
# Run all tests
./scripts/test.sh

# Run specific test categories
pytest tests/unit
pytest tests/integration
```

### Code Style

The project uses:
- Black for code formatting
- Flake8 for style checking
- Ruff for fast linting

### Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Run tests and linting
5. Submit a pull request

## License

[License information]
