#!/bin/bash
set -e  # Exit immediately if a command fails

# Use copy mode for uv to avoid symlinks, which can cause issues on some systems
export UV_LINK_MODE=copy

# Check if Python 3.11 is installed
if ! command -v python3.11 &> /dev/null
then
    echo "ERROR: python3.11 is not found. Please install Python 3.11 or edit setup.sh to use a different version."
    exit 1
fi

# Create a new virtual environment in .venv using Python 3.11
uv venv .venv --python python3.11

# Activate the newly created venv
source .venv/bin/activate

# Install dependencies from pyproject.toml, including dev extras
uv pip install -e ".[dev]" --no-cache

# (Optional) Generate/Update a requirements.txt if you want
# to track pinned versions. This step is optional.
uv pip compile -o requirements.txt pyproject.toml 

echo "Setup complete. Virtual environment is in .venv."
