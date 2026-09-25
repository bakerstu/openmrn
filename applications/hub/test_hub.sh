#!/bin/bash
# Integration test for hub application
# Tests various invocations to catch argument parsing bugs and memory leaks

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HUB_BIN="${SCRIPT_DIR}/targets/linux.x86/hub"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m' # No Color

TESTS_PASSED=0
TESTS_FAILED=0

echo "Hub Application Integration Tests"
echo "=================================="
echo ""

# Helper function to run a test
run_test() {
    local test_name="$1"
    local expected_result="$2"
    shift 2
    local cmd=("$@")
    
    echo -n "Testing: $test_name ... "
    
    if [ "$expected_result" = "fail" ]; then
        # Test should fail (invalid arguments)
        if "${cmd[@]}" 2>/dev/null; then
            echo -e "${RED}FAILED${NC} (should have failed but succeeded)"
            TESTS_FAILED=$((TESTS_FAILED + 1))
            return 1
        else
            echo -e "${GREEN}PASSED${NC}"
            TESTS_PASSED=$((TESTS_PASSED + 1))
            return 0
        fi
    else
        # Test should succeed (valid arguments, but we'll kill it after startup)
        timeout 2 "${cmd[@]}" >/dev/null 2>&1 &
        local pid=$!
        sleep 0.5
        
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
            wait "$pid" 2>/dev/null || true
            echo -e "${GREEN}PASSED${NC}"
            TESTS_PASSED=$((TESTS_PASSED + 1))
            return 0
        else
            echo -e "${RED}FAILED${NC} (process died unexpectedly)"
            TESTS_FAILED=$((TESTS_FAILED + 1))
            return 1
        fi
    fi
}

# Check if hub binary exists
if [ ! -f "$HUB_BIN" ]; then
    echo "Hub binary not found at: $HUB_BIN"
    echo "Please build it first with: cd targets/linux.x86 && make"
    exit 1
fi

# Test 1: Valid port numbers should work
run_test "Valid port 12021" "pass" "$HUB_BIN" -p 12021
run_test "Valid port 8080" "pass" "$HUB_BIN" -p 8080
run_test "Valid port 1" "pass" "$HUB_BIN" -p 1
run_test "Valid port 65535" "pass" "$HUB_BIN" -p 65535

# Test 2: Invalid port numbers should fail
run_test "Invalid port 0" "fail" "$HUB_BIN" -p 0
run_test "Invalid port -1" "fail" "$HUB_BIN" -p -1
run_test "Invalid port 65536" "fail" "$HUB_BIN" -p 65536
run_test "Invalid port 999999" "fail" "$HUB_BIN" -p 999999
run_test "Invalid port 'abc'" "fail" "$HUB_BIN" -p abc
run_test "Invalid port '123abc'" "fail" "$HUB_BIN" -p 123abc

# Test 3: Invalid upstream port numbers should fail
run_test "Invalid upstream port 0" "fail" "$HUB_BIN" -u localhost -q 0
run_test "Invalid upstream port -1" "fail" "$HUB_BIN" -u localhost -q -1
run_test "Invalid upstream port 65536" "fail" "$HUB_BIN" -u localhost -q 65536

# Test 4: Help should work
echo -n "Testing: Help output ... "
if "$HUB_BIN" -h 2>&1 | grep -q "GridConnect CAN HUB"; then
    echo -e "${GREEN}PASSED${NC}"
    TESTS_PASSED=$((TESTS_PASSED + 1))
else
    echo -e "${RED}FAILED${NC}"
    TESTS_FAILED=$((TESTS_FAILED + 1))
fi

# Test 5: Verify no debug output (check for removed fprintf)
echo -n "Testing: No debug output in stderr ... "
output=$("$HUB_BIN" -p 12099 2>&1 &
pid=$!
sleep 0.5
kill $pid 2>/dev/null || true
wait $pid 2>/dev/null || true
)

if echo "$output" | grep -q "packet_printer points to"; then
    echo -e "${RED}FAILED${NC} (debug fprintf detected)"
    TESTS_FAILED=$((TESTS_FAILED + 1))
else
    echo -e "${GREEN}PASSED${NC}"
    TESTS_PASSED=$((TESTS_PASSED + 1))
fi

echo ""
echo "=================================="
echo "Results: $TESTS_PASSED passed, $TESTS_FAILED failed"

if [ $TESTS_FAILED -eq 0 ]; then
    echo -e "${GREEN}All tests passed!${NC}"
    exit 0
else
    echo -e "${RED}Some tests failed!${NC}"
    exit 1
fi
