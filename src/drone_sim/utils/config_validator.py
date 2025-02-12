import json
from pathlib import Path

from jsonschema import validate
import jsonschema.validators

from drone_sim.core.schema_registry import load_schema_mapping

SCHEMA_MAPPING = load_schema_mapping()

def load_schema(schema_name: str) -> dict:
    """Load schema from schemas/ directory with local ref resolution"""
    project_root = Path(__file__).parent.parent.parent.parent
    
    # Use schema name directly from registry including extension
    schema_path = project_root / 'schemas' / schema_name
    
    with open(schema_path) as f:
        schema = json.load(f)
    
    # Resolve local references
    resolver = jsonschema.validators.RefResolver(
        base_uri=f"file://{project_root}/schemas/",
        referrer=schema,
    )
    
    return jsonschema.validators.validator_for(schema)(schema, resolver=resolver).schema

def validate_config(config_data: dict, config_type: str) -> None:
    """Validate config using registry mapping"""
    try:
        schema_name = SCHEMA_MAPPING[config_type]
    except KeyError:
        raise ValueError(f"No schema registered for config type: {config_type}. "
                         f"Available types: {list(SCHEMA_MAPPING.keys())}")
    
    # Add custom validation for new types
    if config_type == 'pathfinding':
        _validate_pathfinding_params(config_data)
    
    # Custom validation logic
    if schema_name == 'collision_strategies':
        _validate_collision_params(config_data)
    
    # Standard JSON schema validation
    schema = load_schema(schema_name)
    validate(instance=config_data, schema=schema)

def _validate_collision_params(config: dict):
    """Additional semantic validation for collision config"""
    active_strategy = config['strategies']['active']
    if active_strategy not in config['strategies']['params']:
        raise ValueError(f"Active strategy {active_strategy} has no parameters defined")
    
    required_params = {
        'hierarchical': ['priority_mode', 'min_speed_factor'],
        'probabilistic': ['risk_threshold']
    }
    
    strategy_params = config['strategies']['params'].get(active_strategy, {})
    for param in required_params.get(active_strategy, []):
        if param not in strategy_params:
            raise ValueError(f"Missing required parameter '{param}' for {active_strategy} strategy")

def _validate_pathfinding_params(config: dict):
    if config['algorithm'] == 'rrt' and 'max_iterations' not in config:
        raise ValueError("RRT algorithm requires max_iterations parameter")

def validate_config_file(file_path: str, config_type: str) -> dict:
    """Validate config file against schema using registry mapping"""
    from pathlib import Path
    
    try:
        schema_file = SCHEMA_MAPPING[config_type]
    except KeyError:
        raise ValueError(f"No schema registered for config type: {config_type}. "
                         f"Available types: {list(SCHEMA_MAPPING.keys())}")
    
    # Get filename from schema registry mapping
    config_file = Path(schema_file).name  # Extract just the filename
    
    try_paths = [
        Path("config") / config_file,
        Path("schemas") / config_file,
        Path(file_path)
    ]
    
    for path in try_paths:
        if path.exists():
            with open(path) as f:
                config_data = json.load(f)
            validate_config(config_data, config_type)
            return config_data
    raise FileNotFoundError(f"No config file found at: {[str(p) for p in try_paths]}")
