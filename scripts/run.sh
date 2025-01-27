#!/bin/bash
cd "$(dirname "$0")/.."  # Move to project root (drone-sim directory)
PYTHONPATH=. python -m src.main data/config/simulation_config.json 