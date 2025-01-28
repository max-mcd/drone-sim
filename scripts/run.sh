#!/bin/bash
cd "$(dirname "$0")/.."  # Move to project root (drone-sim directory)

# Check if --fast flag is passed
if [ "$1" == "--fast" ]; then
    PYTHONPATH=. python -m src.main data/config/simulation_config.json --fast
else
    PYTHONPATH=. python -m src.main data/config/simulation_config.json
fi 