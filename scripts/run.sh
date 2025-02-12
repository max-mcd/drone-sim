#!/bin/bash
cd "$(dirname "$0")/.."

# Fix the venv activation path
if [ -d ".venv" ]; then
    source .venv/bin/activate
else
    echo "Please run: ./scripts/setup.sh first"
    exit 1
fi

# Default values
DEBUG=false
FAST=false
AVOID_COLLISIONS=false

# Default config file
DEFAULT_CONFIG="config/simulation_config.json"

# Parse arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        --config)
            CONFIG_FILE="$2"
            shift 2
            ;;
        --debug) DEBUG=true; shift ;;
        --fast) FAST=true; shift ;;
        --avoid-collisions) AVOID_COLLISIONS=true; shift ;;
        *)
            OTHER_ARGS+=" $1"
            shift
            ;;
    esac
done

# Use default config if none provided
CONFIG_FILE="${CONFIG_FILE:-$DEFAULT_CONFIG}"

# Build command using venv python
CMD_ARGS=()
$FAST && CMD_ARGS+=("--fast")
$DEBUG && CMD_ARGS+=("--debug")
$AVOID_COLLISIONS && CMD_ARGS+=("--avoid-collisions")

# Run with venv python
PYTHONPATH=src .venv/bin/python -m drone_sim.main \
    --config "$CONFIG_FILE" \
    "${CMD_ARGS[@]}" \
    $OTHER_ARGS

