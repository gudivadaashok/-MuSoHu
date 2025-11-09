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

# Simulate some operations
log_info "Performing operation 1..."
sleep 1
log_success "Operation 1 completed successfully"

log_info "Performing operation 2..."
sleep 1
log_warning "Operation 2 completed with warnings"

log_info "Performing operation 3..."
sleep 1
log_error "Operation 3 failed"

log_separator
log_info "Script execution completed"
log_info "Log file: $LOG_FILE"
