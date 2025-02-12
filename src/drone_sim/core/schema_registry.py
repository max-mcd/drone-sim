import json
from pathlib import Path


def load_schema_mapping() -> dict:
    """Load schema mapping from registry file"""
    project_root = Path(__file__).resolve().parent.parent.parent.parent
    registry_path = project_root / 'schemas' / 'schema_registry.json'
    
    with open(registry_path) as f:
        return json.load(f)['mappings']

# Add default schema definitions at the top
DEFAULT_SCHEMAS = {
    "simulation_config": "schemas/simulation_config.json",
    "simulation_main": "schemas/simulation_main.json",
    "collision_system": "schemas/collision_system.json"  # Added missing mapping
} 