# Hub Application Testing Configuration
# Include this in the main Makefile to add test targets

.PHONY: hub-test hub-memcheck hub-test-all

# Run unit tests for hub argument parsing
hub-test:
	@echo "Running hub unit tests..."
	@cd ../../targets/test && ./applications/hub/hub_args_test.test || (echo "Unit tests failed"; exit 1)
	@echo "Unit tests passed!"

# Run integration tests
hub-integration-test:
	@echo "Running hub integration tests..."
	@cd applications/hub && ./test_hub.sh
	@echo "Integration tests passed!"

# Run memory leak detection (requires valgrind)
hub-memcheck:
	@echo "Running memory leak detection..."
	@cd applications/hub && ./test_memcheck.sh

# Run all hub tests
hub-test-all: hub-test hub-integration-test hub-memcheck
	@echo ""
	@echo "==================================="
	@echo "All hub tests passed successfully!"
	@echo "==================================="

# Static analysis checks for common bug patterns
hub-static-check:
	@echo "Running static checks for bug patterns..."
	@echo -n "  Checking for debug fprintf statements... "
	@if grep -r "fprintf.*packet_printer.*points to" applications/hub/*.cxx 2>/dev/null; then \
		echo "FAILED: Debug fprintf found!"; \
		exit 1; \
	else \
		echo "OK"; \
	fi
	@echo -n "  Checking for unsafe atoi() usage... "
	@if grep -E "atoi\s*\(" applications/hub/main.cxx 2>/dev/null | grep -v "test" | grep -v "//"; then \
		echo "FAILED: atoi() found (use strtol instead)!"; \
		exit 1; \
	else \
		echo "OK"; \
	fi
	@echo -n "  Checking for pthread without detach/join... "
	@if grep "pthread_create" applications/hub/targets/*/AvaHiMDNS.cxx 2>/dev/null | grep -v "test"; then \
		if ! grep "pthread_detach\|pthread_join" applications/hub/targets/*/AvaHiMDNS.cxx 2>/dev/null | grep -q .; then \
			echo "FAILED: pthread_create without detach/join!"; \
			exit 1; \
		else \
			echo "OK"; \
		fi \
	else \
		echo "OK (no pthread_create found)"; \
	fi
	@echo -n "  Checking for missing error checks on system calls... "
	@if grep -E "(sem_init|pthread_create|sem_wait)\s*\(" applications/hub/targets/*/AvaHiMDNS.cxx 2>/dev/null | grep -v "result = " | grep -v "if " | grep -v "test"; then \
		echo "FAILED: System call without error check!"; \
		exit 1; \
	else \
		echo "OK"; \
	fi
	@echo "All static checks passed!"

# Add to CI: This should be run as part of continuous integration
hub-ci-test: hub-static-check hub-test hub-integration-test
	@echo "CI checks complete!"
