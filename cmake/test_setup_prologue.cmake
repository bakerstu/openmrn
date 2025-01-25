# This is a target file designed to run on a host system.

# Default coverage option to off.
option(COVERAGE "COVERAGE" OFF)

# Enable the required language standards.
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

if(NOT DEFINED GTEST_TAG)
set(GTEST_TAG v1.15.2)
endif()
message(STATUS "Using GoogleTest tag: ${GTEST_TAG}")

# Get a copy of GoogleTest.
include(FetchContent)
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/${GTEST_TAG}.zip
)

# On Windows: Prevent overriding the parent project's compiler/linker settings.
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# This magic is required in order to properly add the "-x" option for the
# *.cxxtest file extension.
set(CMAKE_CXX_COMPILE_OBJECT
    "<CMAKE_CXX_COMPILER> -c -MD -MF $$@.dep <DEFINES> <INCLUDES> <FLAGS> -o \
    <OBJECT> -c -x c++ <SOURCE>"
)

# Compile flags.
add_compile_options(
    -fPIC -g -O0 -Wall -Werror -DGTEST -D_GNU_SOURCE
    -D__STDC_FORMAT_MACROS -D__STDC_LIMIT_MACROS
    -Wno-attributes
)
if(${COVERAGE})
# Compile flags for coverage.
add_compile_options(--coverage)
endif()

# GTest and GMock includes.
include_directories(${gtest_SOURCE_DIR}/include ${gmock_SOURCE_DIR}/include)

# Add OpenMRN sources
add_subdirectory(${OPENMRNPATH}/src openmrn)

