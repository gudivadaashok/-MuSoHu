#!/bin/bash

#***********************************************************************
# Example script showing how to use the logging configuration
#***********************************************************************

# Source the logging configuration
SCRIPT_DIR="$(dirname "$0")"
source "${SCRIPT_DIR}/logging_config.sh"

# Example usage
log_separator
log_info "Starting example script..."
log_debug "Script initialization complete"

# Simulate some operations
log_debug "Beginning operation 1"
log_info "Performing operation 1..."
sleep 1
log_debug "Operation 1 processing..."
log_success "Operation 1 completed successfully"

log_debug "Beginning operation 2"
log_info "Performing operation 2..."
sleep 1
log_debug "Operation 2 encountered minor issues"
log_warning "Operation 2 completed with warnings"

log_debug "Beginning operation 3"
log_info "Performing operation 3..."
sleep 1
log_debug "Operation 3 failed due to missing dependency"
log_error "Operation 3 failed"

log_separator
log_debug "Cleaning up resources"
log_info "Script execution completed"
log_info "Log file: $LOG_FILE"
log_debug "Debug logging was enabled: $DEBUG_ENABLED"
