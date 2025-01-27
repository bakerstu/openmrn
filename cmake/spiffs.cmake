# Find SPIFFS

if(DEFINED ENV{SPIFFSPATH})
set(SPIFFSPATH $ENV{SPIFFSPATH})
else()
find_path(
    SPIFFSPATH
        src/
    HINTS
        /opt/spiffs
        /opt/spiffs/default
)
endif()

# SPIFFS
if(${TARGET_SPIFFS})
add_library(spiffs
    ${SPIFFSPATH}/src/spiffs_cache.c
    ${SPIFFSPATH}/src/spiffs_check.c
    ${SPIFFSPATH}/src/spiffs_gc.c
    ${SPIFFSPATH}/src/spiffs_hydrogen.c
    ${SPIFFSPATH}/src/spiffs_nucleus.c
    ${OPENMRNPATH}/src/freertos_drivers/spiffs/SPIFFS.cxx
)
target_include_directories(spiffs
    PUBLIC
        ${SPIFFSPATH}/src
        ${OPENMRNPATH}/src/frertos_drivers/spiffs
)
target_compile_options(spiffs
    PUBLIC
        -DNO_TEST
)
endif() # TARGET_SPIFFS
