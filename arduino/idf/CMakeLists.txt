set(SOURCE_DIRS
    src/dcc
    src/executor
    src/freertos_drivers/esp32
    src/freertos_drivers/arduino
    src/openlcb
    src/os
    src/utils
)

if(CONFIG_ENABLE_ARDUINO_DEPENDS)
    list(APPEND SOURCE_DIRS src)
endif()

set(IDF_DEPS
    app_update
    bootloader_support
    driver
    esp_adc
    esp_app_format
    esp_system
    esp_wifi
    espcoredump
    hal
    heap
    vfs
    mdns)

idf_component_register(SRC_DIRS "${SOURCE_DIRS}"
                       INCLUDE_DIRS "src"
                       REQUIRES "${IDF_DEPS}")

###############################################################################
# Suppress compilation warnings in OpenMRN
###############################################################################

target_compile_options(${COMPONENT_LIB} PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-Wno-volatile>)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-type-limits)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-ignored-qualifiers)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-missing-field-initializers)
target_compile_options(${COMPONENT_LIB} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-Wno-class-memaccess>)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-implicit-fallthrough)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error=nonnull)
target_compile_options(${COMPONENT_LIB} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-Wno-redundant-move>)
