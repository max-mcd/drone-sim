import json
import os
from pathlib import Path
from jsonschema import validate
from typing import Any

def load_schema(schema_name: str) -> dict:
    """Load a JSON schema file from the schemas directory."""
    schema_path = Path(__file__).parent.parent / 'schemas' / f'{schema_name}.json'
    with open(schema_path) as f:
        return json.load(f)

def validate_city_config(config_data: dict) -> None:
    """Validate city configuration data against the cities schema."""
    schema = load_schema('cities')
    validate(instance=config_data, schema=schema)

def validate_config_file(file_path: str, schema_name: str) -> dict:
    """Load and validate any configuration file against a named schema."""
    with open(file_path) as f:
        config_data = json.load(f)
    schema = load_schema(schema_name)
    validate(instance=config_data, schema=schema)
    return config_data