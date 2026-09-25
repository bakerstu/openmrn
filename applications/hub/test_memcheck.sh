#!/bin/bash
# Memory leak detection script for hub application
# Uses valgrind to detect memory leaks, particularly in mDNS functions

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HUB_BIN="${SCRIPT_DIR}/targets/linux.x86/hub"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "Hub Application Memory Leak Detection"
echo "======================================"
echo ""

# Check if valgrind is installed
if ! command -v valgrind &> /dev/null; then
    echo -e "${YELLOW}WARNING:${NC} valgrind not found. Install with: sudo apt-get install valgrind"
    echo "Skipping memory leak tests..."
    exit 0
fi

# Check if hub binary exists
if [ ! -f "$HUB_BIN" ]; then
    echo "Hub binary not found at: $HUB_BIN"
    echo "Please build it first with: cd targets/linux.x86 && make"
    exit 1
fi

# Temporary files for valgrind output
VALGRIND_LOG=$(mktemp)
trap "rm -f $VALGRIND_LOG" EXIT

echo "Running hub with valgrind (this will take a few seconds)..."
echo ""

# Run hub with valgrind for 3 seconds to check for leaks
# We disable mDNS to avoid Avahi-related suppressions
timeout 3 valgrind \
    --leak-check=full \
    --show-leak-kinds=all \
    --track-origins=yes \
    --error-exitcode=1 \
    --log-file="$VALGRIND_LOG" \
    "$HUB_BIN" -p 12099 2>&1 >/dev/null || true

echo "Valgrind analysis complete. Checking results..."
echo ""

# Check for memory leaks
if grep -q "definitely lost:" "$VALGRIND_LOG" | grep -v " 0 bytes"; then
    echo -e "${RED}MEMORY LEAKS DETECTED:${NC}"
    echo ""
    grep -A 5 "definitely lost:" "$VALGRIND_LOG" || true
    echo ""
    echo "Full log saved to: $VALGRIND_LOG"
    trap - EXIT  # Don't delete log file on error
    exit 1
fi

if grep -q "indirectly lost:" "$VALGRIND_LOG" | grep -v " 0 bytes"; then
    echo -e "${YELLOW}WARNING: Indirect memory leaks detected${NC}"
    grep -A 5 "indirectly lost:" "$VALGRIND_LOG" || true
    echo ""
fi

# Check for pthread resource leaks
if grep -q "still reachable:" "$VALGRIND_LOG" | grep -v " 0 bytes"; then
    echo -e "${YELLOW}INFO: Some memory still reachable (may be global state)${NC}"
fi

# Summarize results
echo "Memory leak summary:"
grep "LEAK SUMMARY:" -A 5 "$VALGRIND_LOG" || true
echo ""

# Check if all tests passed
if ! grep -q "ERROR SUMMARY: 0 errors" "$VALGRIND_LOG"; then
    echo -e "${RED}FAILED: Valgrind detected errors${NC}"
    echo "Full log: $VALGRIND_LOG"
    trap - EXIT  # Don't delete log file on error
    exit 1
fi

echo -e "${GREEN}SUCCESS: No memory leaks detected!${NC}"
exit 0
