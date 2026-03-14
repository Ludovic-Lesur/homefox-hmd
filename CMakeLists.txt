#
# CMakeLists.txt
#
#  Created on: 14 mar. 2026
#      Author: Ludo
#

# Minimum CMake version.
cmake_minimum_required(VERSION 3.23)

# Project creation.
project(homefox-hmd)
add_executable(${PROJECT_NAME})

# Use object build mode in all sub-directories.
set(BUILD_MODE "OBJECT")

# Macro to convert options to variables.
macro(add_compilation_flag FLAG_NAME FLAG_DESCRIPTION FLAG_DEFAULT_VALUE)
    option(FLAG_NAME FLAG_DESCRIPTION FLAG_DEFAULT_VALUE)
    set(${FLAG_NAME} ${FLAG_DEFAULT_VALUE} CACHE STRING ${FLAG_DESCRIPTION})
    list(APPEND COMPILATION_FLAGS_LIST ${FLAG_NAME})
endmacro()

# Hardware compilation flags.
set(HMD_HW_VERSION "NONE" CACHE STRING "Hardware version")

# Software compilation flags.
add_compilation_flag(HMD_MODE_CLI "Enable command line mode." OFF)
add_compilation_flag(HMD_BUTTON_ENABLE "Enable button." ON)
add_compilation_flag(HMD_TEMPERATURE_HUMIDITY_SHT3X_ENABLE "Enable temperature and humidity measurements with SHT3x sensor." ON)
add_compilation_flag(HMD_TEMPERATURE_HUMIDITY_ENS21X_ENABLE "Enable temperature and humidity measurements with ENS21x sensor." ON)
add_compilation_flag(HMD_AIR_QUALITY_ENABLE "Enable air quality measurements." ON)
add_compilation_flag(HMD_ACCELEROMETER_ENABLE "Enable motion detection." ON)

# Hardware specific settings.
# HMD HW1.0.
if(HMD_HW_VERSION STREQUAL "HW1_0")   
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mcpu=cortex-m0plus -O1")
    set(HMD_MCU "stm32l0xx")
    set(PROJECT_LINKER_SCRIPT "stm32l051x8xx.ld")
# HMD HW2.0.
elseif(HMD_HW_VERSION STREQUAL "HW2_0")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mcpu=cortex-m0plus -O1")
    set(HMD_MCU "stm32l0xx")
    set(PROJECT_LINKER_SCRIPT "stm32l051x8xx.ld")
# Unknown version.
else()
    message(FATAL_ERROR "Invalid hardware version.")
endif()
set(PROJECT_LINKER_PATH "${CMAKE_CURRENT_SOURCE_DIR}/drivers/device/${HMD_MCU}-device/linker")

# Add hardware compilation flags.
add_compile_definitions(
    __HMD_FLAGS_H__
    ${HMD_HW_VERSION}
)

# Add software compilation flags.
foreach(FLAG ${COMPILATION_FLAGS_LIST})
    # Macros only defined.
    if(${FLAG} STREQUAL ON)
        add_compile_definitions(${FLAG})
    endif()
    # Macros with value.
    if((NOT ${FLAG} STREQUAL OFF) AND (NOT ${FLAG} STREQUAL ON))
        set(${FLAG} "(${${FLAG}})")
        add_compile_definitions(${FLAG}=${${FLAG}}) 
    endif()
endforeach()

# Add project sources files.
target_sources(${PROJECT_NAME}
    PRIVATE
        drivers/peripherals/src/mcu_mapping.c
        drivers/components/src/button.c
        drivers/components/src/ens16x_hw.c
        drivers/components/src/ens21x_hw.c
        drivers/components/src/fxls89xxxx_configuration.c
        drivers/components/src/fxls89xxxx_hw.c
        drivers/components/src/led.c
        drivers/components/src/sensors_hw.c
        drivers/components/src/sht3x_hw.c
        drivers/components/src/sx126x_hw.c
        drivers/utils/src/terminal_hw.c
        middleware/analog/src/analog.c
        middleware/cli/src/cli.c
        middleware/power/src/power.c
        middleware/sigfox/sigfox-ep-lib/src/core/TI_aes_128_encr_only.c
        middleware/sigfox/src/mcu_api.c
        middleware/sigfox/src/rf_api.c
        middleware/sigfox/src/rfe.c
        application/src/main.c
)

# Project include folders.
include_directories(${PROJECT_NAME}
    PRIVATE
        drivers/device/inc
        drivers/registers/inc
        drivers/registers/${HMD_MCU}-registers/inc
        drivers/peripherals/inc
        drivers/peripherals/${HMD_MCU}-drivers/inc
        drivers/utils/inc
        drivers/utils/embedded-utils/inc
        drivers/components/inc
        drivers/components/ens16x-driver/inc
        drivers/components/ens21x-driver/inc
        drivers/components/fxls89xxxx-driver/inc
        drivers/components/sht3x-driver/inc
        drivers/components/sx126x-driver/inc
        middleware/analog/inc
        middleware/cli/inc
        middleware/power/inc
        middleware/sigfox/inc
        middleware/sigfox/sigfox-ep-lib/inc
        middleware/sigfox/sigfox-ep-addon-rfp/inc
        application/inc
)

# Add submodules sources files.
add_subdirectory(drivers/device/${HMD_MCU}-device EXCLUDE_FROM_ALL)
add_subdirectory(drivers/peripherals/${HMD_MCU}-drivers EXCLUDE_FROM_ALL)
add_subdirectory(drivers/utils/embedded-utils EXCLUDE_FROM_ALL)
add_subdirectory(drivers/components/ens16x-driver EXCLUDE_FROM_ALL)
add_subdirectory(drivers/components/ens21x-driver EXCLUDE_FROM_ALL)
add_subdirectory(drivers/components/fxls89xxxx-driver EXCLUDE_FROM_ALL)
add_subdirectory(drivers/components/sht3x-driver EXCLUDE_FROM_ALL)
add_subdirectory(drivers/components/sx126x-driver EXCLUDE_FROM_ALL)
add_subdirectory(middleware/sigfox/sigfox-ep-lib EXCLUDE_FROM_ALL)
add_subdirectory(middleware/sigfox/sigfox-ep-addon-rfp EXCLUDE_FROM_ALL)

# Link libraries.
target_link_libraries(${PROJECT_NAME}
    PRIVATE
        ${HMD_MCU}-device
        ${HMD_MCU}-drivers
        embedded-utils
        ens16x-driver
        ens21x-driver
        fxls89xxxx-driver
        sht3x-driver
        sx126x-driver
        sigfox_ep_lib_obj
        sigfox_ep_addon_rfp_obj
        c_nano
        nosys
        gcc
)

# Linker and artifact.
include(script/cmake-arm-none-eabi/linker.cmake)
include(script/cmake-arm-none-eabi/artifact.cmake)
