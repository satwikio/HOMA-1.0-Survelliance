#!/bin/bash
# Run Payload Drone 1 (austin-payload-01)

export DRONE_ID="austin-recon-01"
export SECRET_KEY="drone_aus_123"
export DRONE_TYPE="recon"

# export BACKEND_URL="http://192.168.1.54:8001"  
# export WS_URL="ws://192.168.1.54:8001" 
export BACKEND_URL="http://100.111.237.50:8001"  
export WS_URL="ws://100.111.237.50:8001"
# Get the directory of this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"



# #For robolab
# export BACKEND_URL="http://192.168.1.44:8001"  
# export WS_URL="ws://192.168.1.44:8001"


# Satwik laptop
# export BACKEND_URL="http://100.88.6.111:8001"  
# export WS_URL="ws://100.88.6.111:8001"


# Run the auto-restart wrapper
exec "${SCRIPT_DIR}/run_drone.sh"