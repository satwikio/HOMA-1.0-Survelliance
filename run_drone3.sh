#!/bin/bash
# Run Drone 3 (austin-recon-03)

export DRONE_ID="austin-recon-03"
export SECRET_KEY="drone_aus_123"
export BACKEND_URL = "http://192.168.1.64:8000"  
export WS_URL = "ws://192.168.1.64:8000" 

python3 dummy_script.py