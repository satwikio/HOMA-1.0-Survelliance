#!/bin/bash

#############################################################################
# Drone Script Auto-Restart Wrapper
#
# Features:
# - Automatically restarts script.py on any error/crash
# - Waits for backend to be available before connecting
# - Kills old processes before starting new ones
# - Logs all activity with timestamps
#
# Usage:
#   ./run_drone.sh                    # Uses environment variables or defaults
#   DRONE_ID=drone-01 ./run_drone.sh  # Override via environment
#############################################################################

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Auto-detect Python script (try script.py, then aruco_.py)
if [ -f "${SCRIPT_DIR}/drone.py" ]; then
    SCRIPT_PATH="${SCRIPT_DIR}/drone.py"
# elif [ -f "${SCRIPT_DIR}/aruco_.py" ]; then
#     SCRIPT_PATH="${SCRIPT_DIR}/aruco_.py"
else
    echo "ERROR: Could not find script.py or aruco_.py in ${SCRIPT_DIR}"
    exit 1
fi

LOG_DIR="${SCRIPT_DIR}/logs"

# Drone configuration (read from environment or use defaults)
export DRONE_ID="${DRONE_ID:-austin-recon-01}"
export DRONE_TYPE="${DRONE_TYPE:-recon}"
export STREAM_TYPE="${STREAM_TYPE:-direct}"  # "direct" or "rf_relay"
export SECRET_KEY="${SECRET_KEY:-drone_aus_123}"
export BACKEND_URL="${BACKEND_URL:-http://localhost:8001}"
export WS_URL="${WS_URL:-ws://localhost:8001}"
export USE_DUMMY_TELEMETRY="${USE_DUMMY_TELEMETRY:-True}"
export DETECTION_MODE="${DETECTION_MODE:-aruco}"

# Log file and PID file based on drone ID AND stream type
LOG_FILE="${LOG_DIR}/${DRONE_ID}_${STREAM_TYPE}_$(date +%Y%m%d).log"
PID_FILE="${SCRIPT_DIR}/.drone_script_${DRONE_ID}_${STREAM_TYPE}.pid"

# Activate virtual environment - try multiple locations
VENV_ACTIVATED=false

# Try 1: ../venv (local development)
if [ -f "${SCRIPT_DIR}/../venv/bin/activate" ]; then
    source "${SCRIPT_DIR}/../venv/bin/activate"
    VENV_ACTIVATED=true
# Try 2: ../../homa1_venv (onboard system)
elif [ -f "${SCRIPT_DIR}/../../homa1_venv/bin/activate" ]; then
    source "${SCRIPT_DIR}/../../homa1_venv/bin/activate"
    VENV_ACTIVATED=true
# Try 3: /home/homa1/homa1/homa1_venv (absolute path onboard)
elif [ -f "/home/homa1/homa1/homa1_venv/bin/activate" ]; then
    source "/home/homa1/homa1/homa1_venv/bin/activate"
    VENV_ACTIVATED=true
fi

# Backend settings
MAX_BACKEND_WAIT_RETRIES=0  # 0 means infinite retries
BACKEND_CHECK_INTERVAL=5    # seconds between backend checks

# Restart settings
RESTART_DELAY=3             # seconds to wait before restarting after crash
MAX_CONSECUTIVE_FAILURES=3  # max failures before longer delay
FAILURE_BACKOFF=30          # longer delay after multiple failures

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

#############################################################################
# Logging Functions
#############################################################################

log() {
    local level="$1"
    shift
    local message="$*"
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    echo -e "${timestamp} [${level}] ${message}" | tee -a "${LOG_FILE}"
}

log_info() {
    log "INFO" "${BLUE}$*${NC}"
}

log_success() {
    log "SUCCESS" "${GREEN}$*${NC}"
}

log_warn() {
    log "WARN" "${YELLOW}$*${NC}"
}

log_error() {
    log "ERROR" "${RED}$*${NC}"
}

#############################################################################
# Process Management
#############################################################################

kill_existing_processes() {
    log_info "Checking for existing drone script processes..."

    # Kill process from PID file if it exists
    if [ -f "${PID_FILE}" ]; then
        OLD_PID=$(cat "${PID_FILE}")
        if ps -p "${OLD_PID}" > /dev/null 2>&1; then
            log_warn "Found existing process (PID: ${OLD_PID}), killing it..."
            kill -9 "${OLD_PID}" 2>/dev/null || true
            sleep 1
            log_success "Old process killed"
        fi
        rm -f "${PID_FILE}"
    fi

    # Find and kill any script.py or aruco_.py processes
    EXISTING_PIDS=$(pgrep -f "python.*\(script\.py\|aruco_\.py\)" || true)
    if [ -n "${EXISTING_PIDS}" ]; then
        log_warn "Found running drone script processes: ${EXISTING_PIDS}"
        echo "${EXISTING_PIDS}" | xargs kill -9 2>/dev/null || true
        sleep 1
        log_success "Killed all existing drone script processes"
    else
        log_info "No existing processes found"
    fi
}

#############################################################################
# Backend Health Check
#############################################################################

check_backend_health() {
    # Try to reach backend - more lenient checking for cross-network scenarios

    # Method 1: Try /health endpoint (returns JSON)
    if curl -s -m 5 "${BACKEND_URL}/health" 2>/dev/null | grep -q "healthy"; then
        return 0
    fi

    # Method 2: Try root endpoint (may return 404 but server is UP)
    local http_code=$(curl -s -o /dev/null -w "%{http_code}" -m 5 "${BACKEND_URL}/" 2>/dev/null)
    if [ -n "$http_code" ] && [ "$http_code" != "000" ]; then
        # Any HTTP response code (even 404) means backend is reachable
        return 0
    fi

    # Method 3: Try WebSocket URL (check if port is open)
    local ws_host=$(echo "${WS_URL}" | sed 's|ws://||' | sed 's|wss://||' | cut -d':' -f1)
    local ws_port=$(echo "${WS_URL}" | sed 's|ws://||' | sed 's|wss://||' | cut -d':' -f2 | cut -d'/' -f1)
    if command -v nc >/dev/null 2>&1; then
        if nc -z -w 3 "$ws_host" "$ws_port" 2>/dev/null; then
            return 0
        fi
    fi

    return 1
}

wait_for_backend() {
    log_info "Checking if backend is available at ${BACKEND_URL}..."
    log_info "WebSocket URL: ${WS_URL}"

    local retry_count=0
    while true; do
        if check_backend_health; then
            log_success "Backend is online and reachable!"
            return 0
        fi

        retry_count=$((retry_count + 1))

        if [ ${MAX_BACKEND_WAIT_RETRIES} -gt 0 ] && [ ${retry_count} -ge ${MAX_BACKEND_WAIT_RETRIES} ]; then
            log_error "Backend not available after ${retry_count} attempts. Exiting."
            return 1
        fi

        # Show diagnostic info on first few attempts
        if [ ${retry_count} -le 3 ]; then
            log_warn "Backend not reachable (attempt ${retry_count})"
            log_info "Trying: curl ${BACKEND_URL}/health"
            log_info "Retrying in ${BACKEND_CHECK_INTERVAL}s..."
        else
            log_warn "Backend not available (attempt ${retry_count}). Retrying in ${BACKEND_CHECK_INTERVAL}s..."
        fi

        sleep ${BACKEND_CHECK_INTERVAL}
    done
}

#############################################################################
# Main Script Runner
#############################################################################

run_drone_script() {
    local consecutive_failures=0

    while true; do
        # Wait for backend to be available
        if ! wait_for_backend; then
            log_error "Cannot reach backend. Exiting."
            exit 1
        fi

        # Kill any existing processes before starting
        kill_existing_processes

        log_info "Starting drone script..."
        log_info "Script path: ${SCRIPT_PATH}"

        # Start the Python script and capture its PID
        # python3 "${SCRIPT_PATH}" &
        python3 "${SCRIPT_PATH}" --mode "${DETECTION_MODE}" &
        SCRIPT_PID=$!
        echo ${SCRIPT_PID} > "${PID_FILE}"

        log_success "Drone script started (PID: ${SCRIPT_PID})"

        # Wait for the process to finish
        wait ${SCRIPT_PID}
        EXIT_CODE=$?

        # Clean up PID file
        rm -f "${PID_FILE}"

        # Check exit status
        if [ ${EXIT_CODE} -eq 0 ]; then
            log_success "Script exited cleanly (exit code: 0)"
            consecutive_failures=0
            # Clean exit, probably intentional - restart after short delay
            sleep ${RESTART_DELAY}

        elif [ ${EXIT_CODE} -eq 10 ]; then
            # Network/video transmission failure
            log_error "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            log_error "🌐 NETWORK/VIDEO TRANSMISSION FAILURE (Exit code: 10)"
            log_error "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            log_warn "Internet may be slow or video frames not sending properly"
            log_info "Waiting 10s for network recovery before restart..."
            sleep 10
            consecutive_failures=$((consecutive_failures + 1))

        elif [ ${EXIT_CODE} -eq 11 ]; then
            # Backend connection lost - DON'T restart until backend is back
            log_error "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            log_error "🔌 BACKEND CONNECTION LOST (Exit code: 11)"
            log_error "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            log_warn "Backend disconnected - will wait for it to come back online"
            log_info "Waiting for backend to become available..."

            # IMPORTANT: Loop back to start which checks backend health
            # This prevents restart until backend is actually reachable
            consecutive_failures=0  # Don't count as failure, backend issue
            continue  # Skip to next iteration (will check backend health)

        elif [ ${EXIT_CODE} -eq 12 ]; then
            # Critical error
            log_error "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            log_error "⚠️  CRITICAL ERROR (Exit code: 12)"
            log_error "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            log_error "Manual intervention may be required"
            log_warn "Waiting ${FAILURE_BACKOFF}s before restart..."
            sleep ${FAILURE_BACKOFF}
            consecutive_failures=$((consecutive_failures + 1))

        else
            # Unknown error
            consecutive_failures=$((consecutive_failures + 1))
            log_error "Script crashed with exit code: ${EXIT_CODE}"
            log_error "Consecutive failures: ${consecutive_failures}"

            # Apply backoff if too many consecutive failures
            if [ ${consecutive_failures} -ge ${MAX_CONSECUTIVE_FAILURES} ]; then
                log_warn "Too many consecutive failures. Waiting ${FAILURE_BACKOFF}s before restart..."
                sleep ${FAILURE_BACKOFF}
                consecutive_failures=0  # Reset counter after backoff
            else
                log_info "Restarting in ${RESTART_DELAY}s..."
                sleep ${RESTART_DELAY}
            fi
        fi

        log_info "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        log_info "Initiating restart cycle..."
    done
}

#############################################################################
# Cleanup Handler
#############################################################################

cleanup() {
    log_warn "Shutdown signal received. Cleaning up..."

    if [ -f "${PID_FILE}" ]; then
        SCRIPT_PID=$(cat "${PID_FILE}")
        if ps -p "${SCRIPT_PID}" > /dev/null 2>&1; then
            log_info "Killing drone script (PID: ${SCRIPT_PID})..."
            kill -TERM "${SCRIPT_PID}" 2>/dev/null || true
            sleep 2
            # Force kill if still running
            if ps -p "${SCRIPT_PID}" > /dev/null 2>&1; then
                kill -9 "${SCRIPT_PID}" 2>/dev/null || true
            fi
        fi
        rm -f "${PID_FILE}"
    fi

    log_success "Cleanup complete. Exiting."
    exit 0
}

#############################################################################
# Main Entry Point
#############################################################################

# Create log directory if it doesn't exist
mkdir -p "${LOG_DIR}"

# Register cleanup handler for SIGINT and SIGTERM
trap cleanup SIGINT SIGTERM

# Print banner
log_info "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
log_info "Drone Auto-Restart Script"
log_info "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
log_info "Drone ID: ${DRONE_ID}"
log_info "Drone Type: ${DRONE_TYPE}"
log_info "Stream Type: ${STREAM_TYPE}"
log_info "Backend URL: ${BACKEND_URL}"
log_info "WebSocket URL: ${WS_URL}"
log_info "Script: ${SCRIPT_PATH}"
log_info "Detection Mode: ${DETECTION_MODE}"
if [ "$VENV_ACTIVATED" = true ]; then
    log_success "Virtual environment activated: ${VIRTUAL_ENV}"
else
    log_warn "No virtual environment activated"
fi
log_info "Log: ${LOG_FILE}"
log_info "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Check if script exists
if [ ! -f "${SCRIPT_PATH}" ]; then
    log_error "Python script not found at ${SCRIPT_PATH}"
    exit 1
fi

# Run the main loop
run_drone_script
