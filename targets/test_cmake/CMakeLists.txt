cmake_minimum_required(VERSION 3.14)

# Default coverage option to off.
option(COVERAGE "COVERAGE" OFF)

# Setup directory paths.
set(OPENMRNPATH ${CMAKE_SOURCE_DIR}/../..)

# Specify project.
project(test)

# Uncomment out the following line to get verbose build output.
#set(CMAKE_VERBOSE_MAKEFILE ON)

# Enable testing.
set(GTEST ON)

# Include tests setup.
include(${OPENMRNPATH}/cmake/test_setup_prologue.cmake)

# Include tests setup.
include(${OPENMRNPATH}/cmake/test_setup_epilogue.cmake)