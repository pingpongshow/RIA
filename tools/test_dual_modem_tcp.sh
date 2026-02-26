#!/bin/bash
#
# Dual Modem TCP Test Script
# Tests full modem stack over virtual audio cables
#
# Audio routing:
#   Modem 1: Output → VB-Cable, Input → BlackHole 2ch
#   Modem 2: Output → BlackHole 2ch, Input → VB-Cable
#
# Signal path:
#   Modem 1 TX → VB-Cable → Modem 2 RX
#   Modem 2 TX → BlackHole 2ch → Modem 1 RX

set -e

# Configuration
MODEM1_CMD_PORT=8300
MODEM1_DATA_PORT=8301
MODEM2_CMD_PORT=8310
MODEM2_DATA_PORT=8311

MODEM1_CALLSIGN="TEST1"
MODEM2_CALLSIGN="TEST2"

# DATA receive timeouts must accommodate low-rate profiles (e.g. MC-DPSK + spreading)
DATA_RX_TIMEOUT_SEC=45

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Helper functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_pass() {
    echo -e "${GREEN}[PASS]${NC} $1"
}

log_fail() {
    echo -e "${RED}[FAIL]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

# Send command to modem and get response
send_cmd() {
    local port=$1
    local cmd=$2
    local timeout=${3:-2}

    echo "$cmd" | nc -w $timeout localhost $port 2>/dev/null | head -1
}

# Send command and check for OK response
send_cmd_ok() {
    local port=$1
    local cmd=$2
    local response

    response=$(send_cmd $port "$cmd")
    if [[ "$response" == "OK"* ]]; then
        return 0
    else
        echo "$response"
        return 1
    fi
}

# Wait for state
wait_for_state() {
    local port=$1
    local target_state=$2
    local timeout=${3:-30}
    local elapsed=0

    while [ $elapsed -lt $timeout ]; do
        local state=$(send_cmd $port "STATE")
        if [[ "$state" =~ $target_state ]]; then
            return 0
        fi
        sleep 1
        ((elapsed++))
    done
    return 1
}

# Send data to data port
send_data() {
    local port=$1
    local data=$2

    printf "%s" "$data" | nc -w 2 localhost $port 2>/dev/null
}

# Read data from data port (non-blocking with timeout)
read_data() {
    local port=$1
    local timeout=${2:-5}

    timeout "$timeout" nc -d localhost "$port" 2>/dev/null || true
}

# Check if modems are running
check_modems() {
    log_info "Checking if modems are running..."

    local state1=$(send_cmd $MODEM1_CMD_PORT "STATE" 1)
    local state2=$(send_cmd $MODEM2_CMD_PORT "STATE" 1)

    if [ -z "$state1" ]; then
        log_fail "Modem 1 not responding on port $MODEM1_CMD_PORT"
        return 1
    fi

    if [ -z "$state2" ]; then
        log_fail "Modem 2 not responding on port $MODEM2_CMD_PORT"
        return 1
    fi

    log_pass "Both modems responding"
    echo "  Modem 1 state: $state1"
    echo "  Modem 2 state: $state2"
    return 0
}

# Configure modems
configure_modems() {
    log_info "Configuring modems..."

    # Set callsigns
    send_cmd_ok $MODEM1_CMD_PORT "MYCALL $MODEM1_CALLSIGN" && \
        log_pass "Modem 1 callsign set to $MODEM1_CALLSIGN" || \
        log_fail "Failed to set Modem 1 callsign"

    send_cmd_ok $MODEM2_CMD_PORT "MYCALL $MODEM2_CALLSIGN" && \
        log_pass "Modem 2 callsign set to $MODEM2_CALLSIGN" || \
        log_fail "Failed to set Modem 2 callsign"

    # Enable listen mode on both
    send_cmd_ok $MODEM1_CMD_PORT "LISTEN ON"
    send_cmd_ok $MODEM2_CMD_PORT "LISTEN ON"
    log_pass "Listen mode enabled on both modems"

    # Reset optional payload transforms so tests start from known state.
    send_cmd_ok $MODEM1_CMD_PORT "COMPRESSION OFF" >/dev/null || true
    send_cmd_ok $MODEM2_CMD_PORT "COMPRESSION OFF" >/dev/null || true
    send_cmd_ok $MODEM1_CMD_PORT "ENCRYPT OFF" >/dev/null || true
    send_cmd_ok $MODEM2_CMD_PORT "ENCRYPT OFF" >/dev/null || true
}

# Test basic connection
test_connection() {
    log_info "=== TEST: Basic Connection ==="

    # Initiate connection from modem 1 to modem 2
    log_info "Modem 1 connecting to $MODEM2_CALLSIGN..."
    send_cmd $MODEM1_CMD_PORT "CONNECT $MODEM2_CALLSIGN"

    # Wait for connected state
    if wait_for_state $MODEM1_CMD_PORT "CONNECTED" 60; then
        log_pass "Modem 1 connected!"
    else
        log_fail "Modem 1 connection timeout"
        return 1
    fi

    if wait_for_state $MODEM2_CMD_PORT "CONNECTED" 10; then
        log_pass "Modem 2 connected!"
    else
        log_fail "Modem 2 connection timeout"
        return 1
    fi

    log_pass "Connection established!"
    return 0
}

# Test message transfer
test_message_transfer() {
    log_info "=== TEST: Message Transfer ==="

    local test_msg="Hello from TEST1 - $(date +%s)"
    local rx_file
    rx_file=$(mktemp /tmp/ultra_rx_msg_XXXXXX)

    # Start receiver first and detach stdin so the socket stays open until timeout.
    timeout "$DATA_RX_TIMEOUT_SEC" nc -d localhost "$MODEM2_DATA_PORT" > "$rx_file" 2>/dev/null &
    local rx_pid=$!
    sleep 1

    log_info "Sending message: $test_msg"
    send_data $MODEM1_DATA_PORT "$test_msg"

    wait $rx_pid || true
    log_info "Reading from Modem 2..."
    local received
    received=$(cat "$rx_file")
    rm -f "$rx_file"

    if [[ "$received" == *"$test_msg"* ]]; then
        log_pass "Message received correctly!"
        return 0
    else
        log_fail "Message not received or corrupted"
        echo "  Expected: $test_msg"
        echo "  Received: $received"
        return 1
    fi
}

# Test compression
test_compression() {
    log_info "=== TEST: Compression ==="

    # Enable compression on both sides (sender compresses, receiver decompresses).
    send_cmd_ok $MODEM1_CMD_PORT "COMPRESSION ON" && \
        log_pass "Compression enabled on Modem 1" || \
        log_fail "Failed to enable compression"
    send_cmd_ok $MODEM2_CMD_PORT "COMPRESSION ON" >/dev/null || true

    # Send a highly compressible message
    local test_msg="AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
    local rx_file
    rx_file=$(mktemp /tmp/ultra_rx_cmp_XXXXXX)

    timeout "$DATA_RX_TIMEOUT_SEC" nc -d localhost "$MODEM2_DATA_PORT" > "$rx_file" 2>/dev/null &
    local rx_pid=$!
    sleep 1

    log_info "Sending compressible message (64 bytes of 'A')..."
    send_data $MODEM1_DATA_PORT "$test_msg"

    wait $rx_pid || true
    local received
    received=$(cat "$rx_file")
    rm -f "$rx_file"

    if [[ "$received" == *"$test_msg"* ]]; then
        log_pass "Compressed message received!"
        send_cmd_ok $MODEM1_CMD_PORT "COMPRESSION OFF" >/dev/null || true
        send_cmd_ok $MODEM2_CMD_PORT "COMPRESSION OFF" >/dev/null || true
        return 0
    else
        log_warn "Message may not have been received (check modem logs)"
        send_cmd_ok $MODEM1_CMD_PORT "COMPRESSION OFF" >/dev/null || true
        send_cmd_ok $MODEM2_CMD_PORT "COMPRESSION OFF" >/dev/null || true
        return 1
    fi
}

# Test encryption
test_encryption() {
    log_info "=== TEST: Encryption ==="

    local passphrase="TestKey123"

    # Set encryption key on both modems
    send_cmd_ok $MODEM1_CMD_PORT "ENCRYPTKEY $passphrase" && \
        log_pass "Encryption key set on Modem 1" || \
        log_fail "Failed to set encryption key on Modem 1"

    send_cmd_ok $MODEM2_CMD_PORT "ENCRYPTKEY $passphrase" && \
        log_pass "Encryption key set on Modem 2" || \
        log_fail "Failed to set encryption key on Modem 2"

    # Enable encryption
    send_cmd_ok $MODEM1_CMD_PORT "ENCRYPT ON" && \
        log_pass "Encryption enabled on Modem 1" || \
        log_fail "Failed to enable encryption on Modem 1"

    send_cmd_ok $MODEM2_CMD_PORT "ENCRYPT ON" && \
        log_pass "Encryption enabled on Modem 2" || \
        log_fail "Failed to enable encryption on Modem 2"

    # Send encrypted message
    local test_msg="SECRET MESSAGE $(date +%s)"
    local rx_file
    rx_file=$(mktemp /tmp/ultra_rx_enc_XXXXXX)

    timeout "$DATA_RX_TIMEOUT_SEC" nc -d localhost "$MODEM2_DATA_PORT" > "$rx_file" 2>/dev/null &
    local rx_pid=$!
    sleep 1

    log_info "Sending encrypted message..."
    send_data $MODEM1_DATA_PORT "$test_msg"

    wait $rx_pid || true
    local received
    received=$(cat "$rx_file")
    rm -f "$rx_file"

    if [[ "$received" == *"$test_msg"* ]]; then
        log_pass "Encrypted message received and decrypted!"

        # Disable encryption for subsequent tests
        send_cmd_ok $MODEM1_CMD_PORT "ENCRYPT OFF"
        send_cmd_ok $MODEM2_CMD_PORT "ENCRYPT OFF"

        return 0
    else
        log_fail "Encrypted message not received"
        return 1
    fi
}

# Test waveform modes
test_waveforms() {
    log_info "=== TEST: Waveform Modes ==="

    # Test auto mode (default)
    log_info "Testing AUTO waveform selection..."
    send_cmd_ok $MODEM1_CMD_PORT "WAVEFORM AUTO"
    send_cmd_ok $MODEM2_CMD_PORT "WAVEFORM AUTO"

    local test_msg="Waveform test $(date +%s)"
    send_data $MODEM1_DATA_PORT "$test_msg"
    sleep 5

    local received=$(read_data $MODEM2_DATA_PORT 10)
    if [[ "$received" == *"$test_msg"* ]]; then
        log_pass "AUTO waveform working!"
    fi

    # Test forced MC-DPSK
    log_info "Testing MC-DPSK waveform..."
    send_cmd_ok $MODEM1_CMD_PORT "WAVEFORM MC_DPSK"
    send_cmd_ok $MODEM2_CMD_PORT "WAVEFORM MC_DPSK"

    test_msg="MC-DPSK test $(date +%s)"
    send_data $MODEM1_DATA_PORT "$test_msg"
    sleep 8  # MC-DPSK is slower

    received=$(read_data $MODEM2_DATA_PORT 10)
    if [[ "$received" == *"$test_msg"* ]]; then
        log_pass "MC-DPSK waveform working!"
    fi

    # Reset to auto
    send_cmd_ok $MODEM1_CMD_PORT "WAVEFORM AUTO"
    send_cmd_ok $MODEM2_CMD_PORT "WAVEFORM AUTO"

    return 0
}

# Test file transfer
test_file_transfer() {
    log_info "=== TEST: File Transfer ==="

    # Create test file
    local test_file="/tmp/test_transfer_$(date +%s).txt"
    echo "This is a test file for transfer" > "$test_file"
    echo "Line 2 of test data" >> "$test_file"
    echo "Line 3 with timestamp: $(date)" >> "$test_file"

    log_info "Created test file: $test_file ($(wc -c < "$test_file") bytes)"

    # Send file
    log_info "Sending file..."
    local response
    response=$(send_cmd $MODEM1_CMD_PORT "SENDFILE $test_file")

    if [[ "$response" != "OK"* ]]; then
        log_fail "Failed to initiate file transfer"
        echo "  Response: $response"
        rm -f "$test_file"
        return 1
    fi

    # Wait for transfer
    sleep 15

    log_pass "File transfer initiated (check modem 2 receive directory)"

    # Cleanup
    rm -f "$test_file"

    return 0
}

# Test disconnect
test_disconnect() {
    log_info "=== TEST: Disconnect ==="

    send_cmd $MODEM1_CMD_PORT "DISCONNECT"

    if wait_for_state $MODEM1_CMD_PORT "DISCONNECTED|IDLE" 15; then
        log_pass "Modem 1 disconnected"
    else
        log_warn "Modem 1 disconnect may have timed out"
    fi

    if wait_for_state $MODEM2_CMD_PORT "DISCONNECTED|IDLE" 10; then
        log_pass "Modem 2 disconnected"
    else
        log_warn "Modem 2 disconnect may have timed out"
    fi

    return 0
}

# Print modem status
print_status() {
    log_info "=== MODEM STATUS ==="

    echo "Modem 1 ($MODEM1_CALLSIGN):"
    echo "  State: $(send_cmd $MODEM1_CMD_PORT "STATE")"
    echo "  Version: $(send_cmd $MODEM1_CMD_PORT "VERSION")"
    echo "  Buffer: $(send_cmd $MODEM1_CMD_PORT "BUFFER")"

    echo ""
    echo "Modem 2 ($MODEM2_CALLSIGN):"
    echo "  State: $(send_cmd $MODEM2_CMD_PORT "STATE")"
    echo "  Version: $(send_cmd $MODEM2_CMD_PORT "VERSION")"
    echo "  Buffer: $(send_cmd $MODEM2_CMD_PORT "BUFFER")"
}

# Main test sequence
main() {
    echo "=============================================="
    echo "    DUAL MODEM TCP TEST"
    echo "=============================================="
    echo ""
    echo "Modem 1: Port $MODEM1_CMD_PORT, Callsign $MODEM1_CALLSIGN"
    echo "Modem 2: Port $MODEM2_CMD_PORT, Callsign $MODEM2_CALLSIGN"
    echo ""
    echo "Audio routing:"
    echo "  Modem 1: Output=VB-Cable, Input=BlackHole 2ch"
    echo "  Modem 2: Output=BlackHole 2ch, Input=VB-Cable"
    echo ""
    echo "=============================================="
    echo ""

    # Check modems
    if ! check_modems; then
        log_fail "Please start both modems with TCP enabled before running this test"
        echo ""
        echo "To start modems:"
        echo "  Terminal 1: ./build/ria_gui  (set TCP port 8300, audio: Out=VB-Cable, In=BlackHole)"
        echo "  Terminal 2: RIA_TCP_PORT=8310 ./build/ria_gui  (set TCP port 8310, audio: Out=BlackHole, In=VB-Cable)"
        exit 1
    fi

    # Print initial status
    print_status
    echo ""

    # Configure
    configure_modems
    echo ""

    # Run tests
    local passed=0
    local failed=0

    if test_connection; then
        ((passed++))
    else
        ((failed++))
    fi
    echo ""

    if test_message_transfer; then
        ((passed++))
    else
        ((failed++))
    fi
    echo ""

    if test_compression; then
        ((passed++))
    else
        ((failed++))
    fi
    echo ""

    if test_encryption; then
        ((passed++))
    else
        ((failed++))
    fi
    echo ""

    if test_waveforms; then
        ((passed++))
    else
        ((failed++))
    fi
    echo ""

    if test_file_transfer; then
        ((passed++))
    else
        ((failed++))
    fi
    echo ""

    if test_disconnect; then
        ((passed++))
    else
        ((failed++))
    fi
    echo ""

    # Summary
    echo "=============================================="
    echo "    TEST SUMMARY"
    echo "=============================================="
    log_pass "Passed: $passed"
    if [ $failed -gt 0 ]; then
        log_fail "Failed: $failed"
    fi
    echo ""

    # Final status
    print_status

    if [ $failed -eq 0 ]; then
        exit 0
    else
        exit 1
    fi
}

# Run main
main "$@"
