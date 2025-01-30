#!/bin/bash
cd "$(dirname "$0")/.."  # Move to project root (drone-sim directory)

# Set Python path
export PYTHONPATH=.

# Default values
DEBUG=false
FAST=false
AVOID_COLLISIONS=false

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --debug)
            DEBUG=true
            shift
            ;;
        --fast)
            FAST=true
            shift
            ;;
        --avoid-collisions)
            AVOID_COLLISIONS=true
            shift
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 [--fast] [--debug] [--avoid-collisions]"
            exit 1
            ;;
    esac
done

# Initialize base command
CMD="python -m src.main data/config/simulation_config.json"

# Process all flags
if [ "$FAST" = true ]; then
    CMD="$CMD --fast"
fi

if [ "$DEBUG" = true ]; then
    CMD="$CMD --debug"
fi

if [ "$AVOID_COLLISIONS" = true ]; then
    CMD="$CMD --avoid-collisions"
fi

# Run the command
if [[ $CMD == *"--debug"* ]]; then
    # If debug flag is present, redirect output to file
    $CMD &> simulation_run_result.txt
    echo "Debug output saved to simulation_run_result.txt"
else
    # Otherwise run normally
    $CMD
fi
