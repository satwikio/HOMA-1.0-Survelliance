#!/bin/bash
# Run Drone 2 (austin-recon-02)

export DRONE_ID="austin-recon-02"
export SECRET_KEY="drone_aus_123"
export BACKEND_URL="http://192.168.1.64:8001"  
export WS_URL="ws://192.168.1.64:8001" 

python3 dummy_script.py