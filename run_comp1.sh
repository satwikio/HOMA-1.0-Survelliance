#!/bin/bash
set -e   # stop on error

# Move into project directory
cd "$HOME/homa1"

# Activate virtual environment
source homa1_venv/bin/activate

# Move into code directory
cd code_base/HOMA-1.0/

# Run the drone script
bash run_drone1.sh "$@"
