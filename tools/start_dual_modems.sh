#!/bin/bash
#
# Start Dual Modems for Testing
#
# This script starts two ria_gui instances with different configurations.
#
# Audio routing:
#   Modem 1: Output → VB-Cable, Input → BlackHole 2ch
#   Modem 2: Output → BlackHole 2ch, Input → VB-Cable
#
# Signal path:
#   Modem 1 TX → VB-Cable → Modem 2 RX
#   Modem 2 TX → BlackHole 2ch → Modem 1 RX

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/../build"

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[0;33m'
NC='\033[0m'

echo -e "${BLUE}=============================================="
echo -e "    STARTING DUAL MODEM INSTANCES"
echo -e "==============================================${NC}"
echo ""

# Check if modems are already running
if nc -z localhost 8300 2>/dev/null; then
    echo -e "${YELLOW}Warning: Modem 1 may already be running on port 8300${NC}"
fi
if nc -z localhost 8310 2>/dev/null; then
    echo -e "${YELLOW}Warning: Modem 2 may already be running on port 8310${NC}"
fi

echo -e "${GREEN}Starting Modem 1 (TEST1)...${NC}"
echo "  Config: ~/.config/ultra/settings_1.ini"
echo "  Audio:  Output=VB-Cable, Input=BlackHole 2ch"
echo "  TCP:    cmd=8300, data=8301"
echo ""

# Start modem 1 in background
ULTRA_INSTANCE=1 "$BUILD_DIR/ria_gui" &
MODEM1_PID=$!
echo "  PID: $MODEM1_PID"

sleep 2

echo ""
echo -e "${GREEN}Starting Modem 2 (TEST2)...${NC}"
echo "  Config: ~/.config/ultra/settings_2.ini"
echo "  Audio:  Output=BlackHole 2ch, Input=VB-Cable"
echo "  TCP:    cmd=8310, data=8311"
echo ""

# Start modem 2 in background
ULTRA_INSTANCE=2 "$BUILD_DIR/ria_gui" &
MODEM2_PID=$!
echo "  PID: $MODEM2_PID"

sleep 2

echo ""
echo -e "${GREEN}Both modems started!${NC}"
echo ""
echo "Modem 1 (TEST1): PID=$MODEM1_PID, TCP cmd=8300, data=8301"
echo "Modem 2 (TEST2): PID=$MODEM2_PID, TCP cmd=8310, data=8311"
echo ""
echo "To run tests:"
echo "  ./tools/test_dual_modem_tcp.sh"
echo ""
echo "To stop modems:"
echo "  kill $MODEM1_PID $MODEM2_PID"
echo "  or: pkill -f ria_gui"
echo ""

# Save PIDs to file for easy cleanup
echo "$MODEM1_PID $MODEM2_PID" > /tmp/ultra_modem_pids.txt

echo "PIDs saved to /tmp/ultra_modem_pids.txt"
echo ""
echo "Press Ctrl+C to stop this script (modems will continue running)"

# Wait for both processes
wait
