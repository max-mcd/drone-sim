#!/bin/bash
cd "$(dirname "$0")/.."  # Move to project root (drone-sim directory)

# Set Python path
export PYTHONPATH=.

# Initialize base command
CMD="python -m src.main data/config/simulation_config.json"

# Process all flags
while [[ $# -gt 0 ]]; do
    case $1 in
        --fast)
            CMD="$CMD --fast"
            shift
            ;;
        --debug)
            CMD="$CMD --debug"
            shift
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 [--fast] [--debug]"
            exit 1
            ;;
    esac
done

# Run the command
if [[ $CMD == *"--debug"* ]]; then
    # If debug flag is present, redirect output to file
    $CMD &> simulation_run_result.txt
    echo "Debug output saved to simulation_run_result.txt"
else
    # Otherwise run normally
    $CMD
fi
