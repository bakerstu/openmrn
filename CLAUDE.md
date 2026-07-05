# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

OpenMRN (Open Model Railroad Network) is a C++ library that implements the NMRA's LCC (Layout Command Control) bus protocol. The project targets embedded systems running on various microcontrollers, as well as Linux/Mac hosts. It uses a sophisticated multi-target build system based on recursive make that allows compiling the same source code for many different platforms.

## Common Development Commands

### Building

```bash
# Build documentation (generates HTML in doc/html)
make docs

# View documentation warnings
make docsw
```

### Testing

```bash
# Run full unit test suite (parallel)
make tests

# Run tests single-threaded (for CI/debugging)
make tests-single

# Run tests on LLVM target
make llvm-tests

# Run tests on JavaScript/Emscripten target
make js-tests

# Run all tests (native + LLVM)
make alltests

# Run tests for a specific module (e.g., openlcb)
make -C targets/test/openlcb tests

# Run a single test file
make -C targets/test tests TESTBLACKLIST="utils/SocketClient.cxxtest"
```

### Building Applications

```bash
# Build a specific application target (faster than building all)
cd applications/hub/targets/linux.x86
make

# Build all applications
make -C applications
```

## Code Organization

### Core Architecture

The build system uses two key concepts:

1. **Core Targets** (`targets/`): Represent OS + processor combinations (e.g., `linux.x86`, `freertos.armv7m`). These compile the core library once.

2. **Application Targets** (`applications/*/targets/`): Each produces a binary for a specific board/platform. They select a core target for linking.

When building an application, both the core target and application target are compiled. The system automatically skips compilation of unchanged code.

### Main Source Directories

- **`src/openlcb/`**: LCC/OpenLCB protocol stack implementation. Uses state flows and can run in single-threaded environments.

- **`src/executor/`**: Core coroutine-based multi-tasking framework. Enables lightweight cooperative multitasking using state machines and message queues.

- **`src/freertos_drivers/`**: Hardware abstraction layer with POSIX-compatible drivers. Organized by vendor:
  - `common/` - Base driver classes
  - `st/`, `nxp/`, `ti/`, `esp8266/`, `pic32mx/` - Vendor-specific implementations
  - `net_*` - Network stack implementations

- **`src/utils/`**: Utility classes and helpers: buffers, queues, logging, streams, sockets, hubs.

- **`src/os/`**: OS abstraction layer providing threads, mutexes, semaphores, timers, GPIO, mDNS across Linux, Mac, FreeRTOS, Windows.

- **`src/dcc/`**: DCC protocol declarations and classes (command stations, decoders, RailCom).

- **`src/console/`**: Telnet-like server for command-line console access.

- **`boards/`**: Hardware-specific code (interrupt vectors, linker scripts, GPIO pinouts) organized by MCU family and board.

- **`applications/`**: Example applications demonstrating the framework.

### Application Structure

A typical application has:
```
myapp/
├── Makefile
├── config.mk              # Build configuration
├── subdirs                # List of application libraries
├── main.cxx               # Application entry point
└── targets/
    ├── Makefile
    ├── linux.x86/         # Build files for specific target
    │   ├── Makefile
    │   ├── config.hxx     # Configuration memory layout
    │   └── hardware.hxx   # Hardware resource declarations
    └── other.target/
```

## Code Style and Standards

### Formatting

- **Tool**: Use `clang-format` for automatic formatting (configured in `.clang-format`)
- **Style**: WebKit-based with Allman-style braces
- **Indentation**: 4 spaces (no tabs, except in Makefiles where required by syntax)
- **Line length**: 80 columns
- **Line endings**: Unix LF only (never CRLF)

### Naming Conventions

- **Classes**: `UpperCamelCase`
- **Functions/methods**: `lower_snake_case`
- **Constants**: `UPPER_SNAKE_CASE`
- **Member variables**: `lower_snake_case_` (with trailing underscore for private)

### Documentation

- **Doxygen comments required** for all public language members (classes, functions, etc.)
- **Style**: Javadoc-style preferred (`/** ... */` or `///`)
- **Tags**: Use `@` instead of `\` for Doxygen commands
- **Don't use** `//!` or `/*!` - these are forbidden for consistency

Example:
```cpp
/// @brief This is a function.
/// @param param The parameter description
/// @return What is returned
int example_function(int param);
```

## Testing

### Test Framework

- **Framework**: Google Test (gtest/gmock)
- **Test files**: Use `.cxxtest` extension
- **Blacklisted tests** (known to be flaky):
  - `utils/SocketClient.cxxtest`
  - `openlcb/IfTcp.cxxtest`

### Writing Tests

Tests are located in `targets/test/<module>/` with a `Makefile` that includes `$(OPENMRNPATH)/etc/lib.mk`. The build system automatically discovers and compiles `.cxxtest` files.

## Important Files and Variables

### Build System

- **`$(OPENMRNPATH)`**: Must point to the OpenMRN root directory (set automatically in-tree)
- **`$(APP_PATH)`**: Locates the application root (for external applications)
- **`etc/*.mk`**: Makefile helper libraries that implement core build logic

### Environment

Some targets require external dependencies (e.g., STM32Cube, FreeRTOS, toolchains). These are discovered via `etc/path.mk` which searches standard installation paths. Override by setting environment variables (e.g., `export FREERTOSDIR=/opt/freertos`).

## Multi-Target Compilation

The codebase supports compilation for:

### Host Targets
- `linux.x86` / `linux.llvm` - Linux PC
- `mach.x86_64` - macOS
- `mingw.x86` - Windows (cross-compiled)

### Embedded Targets
- `freertos.armv7m` - Most common: Cortex-M3/M4 with FreeRTOS
- `freertos.armv6m` - Cortex-M0/M0+ with FreeRTOS
- `freertos.armv4t` - ARM7TDMI with FreeRTOS
- `freertos.mips4k` - PIC32MX with FreeRTOS
- `bare.armv7m` - Cortex-M3/M4 bare metal (no OS)
- `nonos.xtensa-lx106.esp8266` - ESP8266 WiFi

### Special Targets
- `cov` - Code coverage analysis
- `js.emscripten` - JavaScript/Node.js via Emscripten

## Debugging and Troubleshooting

### Build Issues

- **Missing dependencies**: The build will skip targets whose dependencies aren't installed. Check output for messages like "Ignoring target X because library Y is missing."
- **Clean rebuild**: `make clean` (target-specific) or `make veryclean` (entire target)
- **Parallel build failures**: Try `make tests-single` instead of `make tests` to see real errors

### Code Issues

- **Doxygen warnings**: Check `doc/warnings` after building docs
- **Formatting**: Run `clang-format -i <file>` to auto-fix formatting

## Key Implementation Details

### Executor Framework

The `executor` is a lightweight coroutine/state-machine framework used extensively by the OpenLCB stack. It enables complex protocol implementations on single-threaded embedded systems. Most protocol code is implemented as state flows (not free-running threads).

### Direct Hub Pattern

Used for routing CAN frames between different interfaces (USB, network, CAN bus). See `src/utils/DirectHub.md` for details.

### Byte Stream Protocol

LCC uses a byte stream protocol in addition to CAN. See `doc/byte_stream.md` for the specification.

## Continuous Integration

CI runs on GitHub Actions and executes:
```bash
make -j5 tests-single TESTBLACKLIST="utils/SocketClient.cxxtest openlcb/IfTcp.cxxtest"
```

This installs libavahi-client-dev and libssl-dev as dependencies.
