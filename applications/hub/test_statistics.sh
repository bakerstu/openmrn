#!/bin/bash
# Test that hub statistics are properly tracked
#
# This tests that packet and byte counters update correctly when traffic flows through the hub.

set -e

HUB=./hub
TEST_PORT=12099
MON_PORT=8099
TIMEOUT=10

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

pass() {
    echo -e "${GREEN}✓${NC} $1"
}

fail() {
    echo -e "${RED}✗${NC} $1"
    exit 1
}

info() {
    echo -e "${YELLOW}ℹ${NC} $1"
}

cleanup() {
    if [ -n "$HUB_PID" ]; then
        kill $HUB_PID 2>/dev/null || true
        wait $HUB_PID 2>/dev/null || true
    fi
}

trap cleanup EXIT

# Test 1: Statistics start at zero
info "Test 1: Verify initial statistics are zero"
$HUB -p $TEST_PORT -M $MON_PORT &
HUB_PID=$!
sleep 2

# Check if hub started
if ! kill -0 $HUB_PID 2>/dev/null; then
    fail "Hub failed to start"
fi

# Fetch initial statistics
STATS=$(curl -s http://127.0.0.1:$MON_PORT/status)
if [ -z "$STATS" ]; then
    fail "Failed to fetch statistics"
fi

# Parse JSON using simple text processing
# Extract packets.received
PACKETS_RX=$(echo "$STATS" | sed -n 's/.*"packets"[^{]*{[^}]*"received":[[:space:]]*\([0-9]*\).*/\1/p')
PACKETS_TX=$(echo "$STATS" | sed -n 's/.*"packets"[^{]*{[^}]*"sent":[[:space:]]*\([0-9]*\).*/\1/p')
# Extract bytes.received
BYTES_RX=$(echo "$STATS" | sed -n 's/.*"bytes"[^{]*{[^}]*"received":[[:space:]]*\([0-9]*\).*/\1/p')
BYTES_TX=$(echo "$STATS" | sed -n 's/.*"bytes"[^{]*{[^}]*"sent":[[:space:]]*\([0-9]*\).*/\1/p')

if [ "$PACKETS_RX" = "0" ] && [ "$PACKETS_TX" = "0" ]; then
    pass "Initial packet counters are zero (rx=$PACKETS_RX, tx=$PACKETS_TX)"
else
    fail "Initial packet counters not zero (rx=$PACKETS_RX, tx=$PACKETS_TX)"
fi

if [ "$BYTES_RX" = "0" ] && [ "$BYTES_TX" = "0" ]; then
    pass "Initial byte counters are zero (rx=$BYTES_RX, tx=$BYTES_TX)"
else
    fail "Initial byte counters not zero (rx=$BYTES_RX, tx=$BYTES_TX)"
fi

# Test 2: Statistics increment with traffic
info "Test 2: Verify statistics increment with packet traffic"

# Send some GridConnect packets to the hub
# Format: :X<id>N<data>;
# Example: :X1234N0102030405060708;

(
    echo ":X00001234N0102030405060708;"
    sleep 0.1
    echo ":X00005678N1122334455667788;"
    sleep 0.1
    echo ":X0000ABCDN;"
    sleep 0.5
) | nc -w 1 127.0.0.1 $TEST_PORT || true

sleep 1

# Fetch updated statistics
STATS=$(curl -s http://127.0.0.1:$MON_PORT/status)
PACKETS_RX_NEW=$(echo "$STATS" | sed -n 's/.*"packets"[^{]*{[^}]*"received":[[:space:]]*\([0-9]*\).*/\1/p')
BYTES_RX_NEW=$(echo "$STATS" | sed -n 's/.*"bytes"[^{]*{[^}]*"received":[[:space:]]*\([0-9]*\).*/\1/p')

if [ "$PACKETS_RX_NEW" -gt 0 ]; then
    pass "Packet counter incremented (rx=$PACKETS_RX_NEW packets)"
else
    fail "Packet counter did not increment (rx=$PACKETS_RX_NEW)"
fi

if [ "$BYTES_RX_NEW" -gt 0 ]; then
    pass "Byte counter incremented (rx=$BYTES_RX_NEW bytes)"
else
    fail "Byte counter did not increment (rx=$BYTES_RX_NEW bytes)"
fi

# Test 3: Connection statistics
info "Test 3: Verify connection statistics track clients"

# Check current connections
CONNS_BEFORE=$(echo "$STATS" | sed -n 's/.*"connections"[^{]*{[^}]*"active":[[:space:]]*\([0-9]*\).*/\1/p')

# Open a connection and hold it
exec 3<>/dev/tcp/127.0.0.1/$TEST_PORT
sleep 0.5

# Check connections increased
STATS=$(curl -s http://127.0.0.1:$MON_PORT/status)
CONNS_AFTER=$(echo "$STATS" | sed -n 's/.*"connections"[^{]*{[^}]*"active":[[:space:]]*\([0-9]*\).*/\1/p')
CONNS_TOTAL=$(echo "$STATS" | sed -n 's/.*"connections"[^{]*{[^}]*"total":[[:space:]]*\([0-9]*\).*/\1/p')

if [ "$CONNS_AFTER" -gt "$CONNS_BEFORE" ]; then
    pass "Active connection count increased (before=$CONNS_BEFORE, after=$CONNS_AFTER)"
else
    fail "Active connection count did not increase (before=$CONNS_BEFORE, after=$CONNS_AFTER)"
fi

if [ "$CONNS_TOTAL" -gt 0 ]; then
    pass "Total connection count tracked (total=$CONNS_TOTAL)"
else
    fail "Total connection count not tracked"
fi

# Close the connection
exec 3>&-
exec 3<&-
sleep 0.5

# Check active connections decreased
STATS=$(curl -s http://127.0.0.1:$MON_PORT/status)
CONNS_FINAL=$(echo "$STATS" | sed -n 's/.*"connections"[^{]*{[^}]*"active":[[:space:]]*\([0-9]*\).*/\1/p')

if [ "$CONNS_FINAL" -lt "$CONNS_AFTER" ]; then
    pass "Active connection count decreased after disconnect (final=$CONNS_FINAL)"
else
    fail "Active connection count did not decrease (final=$CONNS_FINAL)"
fi

# Test 4: Verify stats dump to stderr
info "Test 4: Verify --stats-dump flag works"
kill $HUB_PID 2>/dev/null || true
wait $HUB_PID 2>/dev/null || true

# Start hub with stats dumping every 2 seconds
$HUB -p $TEST_PORT -M $MON_PORT --stats-dump 2 2>&1 | tee /tmp/hub_stats.log &
HUB_PID=$!
sleep 5

# Check if stats were printed to stderr
if grep -q "HUB STATS" /tmp/hub_stats.log; then
    pass "Stats dump to stderr working"
else
    fail "Stats dump not found in stderr"
fi

# Test 5: Verify monitoring can be disabled
info "Test 5: Verify --no-monitoring flag works"
kill $HUB_PID 2>/dev/null || true
wait $HUB_PID 2>/dev/null || true

$HUB -p $TEST_PORT --no-monitoring &
HUB_PID=$!
sleep 2

# Try to connect to monitoring port (should fail)
if curl -s -m 1 http://127.0.0.1:$MON_PORT/status >/dev/null 2>&1; then
    fail "Monitoring server still running with --no-monitoring"
else
    pass "Monitoring disabled with --no-monitoring flag"
fi

info "All statistics tests passed!"
exit 0
