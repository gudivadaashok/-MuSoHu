#!/bin/bash
source "$(dirname "$0")/logging_config.sh"

echo "Testing logging functions..."
echo "Note: Debug messages are only shown in terminal when DEBUG_ENABLED=true"
echo "Debug messages are always written to the log file."
echo ""

log_debug "Debug message - checking configuration"
log_info "Starting script..."
log_debug "Initializing variables and settings"
log_success "Task completed!"
log_warning "Warning message"
log_debug "Debug: About to simulate an error"
log_error "Error occurred"
log_debug "Debug message - script execution completed"

echo ""
echo "To see debug messages in terminal, run with: DEBUG_ENABLED=true ./test_logging.sh"
echo "Log file location: $LOG_FILE"
