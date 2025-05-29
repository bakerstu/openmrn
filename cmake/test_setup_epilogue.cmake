# This is a target file designed to run on a host system.

# Treat the tests (*.cxxtest extension) as C++ language files.
set_source_files_properties(${TESTS} PROPERTIES LANGUAGE CXX)

if(${COVERAGE})
# Setup gcov target.
add_custom_target(TARGET_GCOV ALL
    DEPENDS gcovr/coverage.html
)
endif()

# Build an executable for each test file.
foreach(testsourcefile ${TESTS})
    # Manipulate path to result in unique test names.
    get_filename_component(testname ${testsourcefile} NAME_WE)
    get_filename_component(testdir ${testsourcefile} DIRECTORY)
    get_filename_component(testdir ${testdir} NAME)
    set(testname "${testdir}_${testname}")

    add_executable(${testname} ${testsourcefile})
    target_include_directories(${testname}
        PUBLIC
            $ENV{OPENMRNPATH}/src
            $ENV{OPENMRNPATH}/include
            $ENV{SXMLCPATH}/src
            ${ROOT_DIR}
    )
    target_link_libraries(${testname}
        GTest::gtest_main
        GTest::gmock_main

        -fPIC
        -lgcov
        -lcrypto
        -Wl,--start-group
        openmrn
        ${LINK_LIBS}
        -Wl,--end-group
        -lavahi-client
        -lavahi-common
    )
    add_custom_command(TARGET ${testname}
        POST_BUILD
        COMMAND ./${testname}
        # Forces the coverage report to be regenerated if tests are run.
        COMMAND rm -rf gcovr
    )
    if(${COVERAGE})
    add_dependencies(TARGET_GCOV ${testname})
    endif()
endforeach(testsourcefile ${TESTS})

if(${COVERAGE})
# Generate the HTML coverage report
add_custom_command(OUTPUT gcovr/coverage.html
    COMMAND mkdir -p gcovr
    COMMAND gcovr --decisions --exclude-unreachable-branches --gcov-ignore-parse-errors negative_hits.warn_once_per_file --html-details gcovr/coverage.html --exclude=_deps.* -r ${GCOV_SOURCE_DIR} .
    VERBATIM
)
endif()