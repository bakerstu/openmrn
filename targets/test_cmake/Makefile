-include ../../config.mk

# Default to run 5 jobs of parallelism.
JOBS ?= 5

# Default (non-coverage) target.
all: .nocov build
	cmake --build build/ -j $(JOBS)

# Coverage target.
.PHONY: cov
cov: .cov build_cov
	cmake --build build/ -j $(JOBS)
	ln -s -f build/gcovr/coverage.html coverage.html

# Default build.
build:
	cmake -S . -B build/

# Coverage build.
build_cov:
	cmake -DCOVERAGE:BOOL=ON -S . -B build/

# Clean.
.PHONY: clean
clean:
	cmake --build build/ --target clean

# Scrub all build artifacts.
.PHONY: veryclean
veryclean:
	rm -rf build/ .cov .nocov coverage.html

# This magic handles switching from nocov to cov.
.cov:
	rm -rf build/ .nocov
	touch .cov

# This magic handles switching from cov to nocov.
.nocov:
	rm -rf build/ .cov coverage.html
	touch .nocov