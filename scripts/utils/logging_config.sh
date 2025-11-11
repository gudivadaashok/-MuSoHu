#!/bin/bash

#***********************************************************************
# Logging Configuration for MuSoHu Scripts
#***********************************************************************

# Log directory - always use scripts/logs regardless of where script is run from
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$(dirname "$SCRIPT_DIR")/logs"
mkdir -p "$LOG_DIR"

# Get the name of the calling script (without path and extension)
CALLING_SCRIPT=$(basename "${0}" .sh)

# Log file with script name and timestamp
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
LOG_FILE="${LOG_DIR}/${CALLING_SCRIPT}_${TIMESTAMP}.log"

# Colors for terminal output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Debug logging control
DEBUG_ENABLED=${DEBUG_ENABLED:-false}

# Common logging variables
LOG_TO_FILE="tee -a \"$LOG_FILE\""
get_timestamp_message() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') - $1"
}

# Logging functions
log_debug() {
    local message="$(get_timestamp_message "$1")"
    if [[ "$DEBUG_ENABLED" == "true" ]]; then
        echo -e "${PURPLE}[DEBUG]${NC} $message" | eval "$LOG_TO_FILE"
    else
        # Only write to log file, not to terminal
        echo "[DEBUG] $message" >> "$LOG_FILE"
    fi
}

log_info() {
    local message="$(get_timestamp_message "$1")"
    echo -e "${BLUE}[INFO]${NC} $message" | eval "$LOG_TO_FILE"
}

log_success() {
    local message="$(get_timestamp_message "$1")"
    echo -e "${GREEN}[SUCCESS]${NC} $message" | eval "$LOG_TO_FILE"
}

log_warning() {
    local message="$(get_timestamp_message "$1")"
    echo -e "${YELLOW}[WARNING]${NC} $message" | eval "$LOG_TO_FILE"
}

log_error() {
    local message="$(get_timestamp_message "$1")"
    echo -e "${RED}[ERROR]${NC} $message" | eval "$LOG_TO_FILE"
}

log_separator() {
    echo "_______________________*******_______________________" | eval "$LOG_TO_FILE"
}

# Export functions and variables
export DEBUG_ENABLED
export LOG_DIR
export LOG_FILE
export LOG_TO_FILE
export -f get_timestamp_message
export -f log_debug
export -f log_info
export -f log_success
export -f log_warning
export -f log_error
export -f log_separator
