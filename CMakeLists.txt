cmake_minimum_required(VERSION 3.22)

include(FetchContent)

# Testing FetchContent
# set(CMAKE_MESSAGE_LOG_LEVEL DEBUG)

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Hexapod)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Include external projects
set(CMSISCORE ${CMAKE_SOURCE_DIR}/Drivers/CMSIS)
FetchContent_Declare(
        cmsis-dsp
        GIT_REPOSITORY https://github.com/ARM-software/CMSIS-DSP
        GIT_TAG main
)
FetchContent_MakeAvailable(cmsis-dsp)

FetchContent_Declare(
        HexapodMath
        GIT_REPOSITORY https://github.com/spark404/HexapodMath.git
        GIT_TAG "beb4118ac1a2d5be3d803ecce902c02d2327fb02"
        GIT_PROGRESS ON
        GIT_REMOTE_UPDATE_STRATEGY CHECKOUT
)
FetchContent_MakeAvailable(HexapodMath)

FetchContent_Declare(
        dynamixel
        GIT_REPOSITORY https://github.com/spark404/dynamixel.git
        GIT_TAG "d60f6e4c55afa583669af5c39c61c88fe5b3d406"
        GIT_PROGRESS ON
        GIT_REMOTE_UPDATE_STRATEGY CHECKOUT
)
FetchContent_MakeAvailable(dynamixel)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user sources here
        Core/Src/calculator.c
        Core/Src/dynamixel_ll_uart.c
        Core/Src/servos.c
        Core/Src/rgb_led.c
        Core/Src/controller.c
        Core/Src/measure.c
        Core/Src/attitude_measurement.c
        Core/Src/attitude_ekf.c
        Core/Src/ring_buffer.c
        Core/Src/error_handling.c
        Drivers/BMI08X/Src/bmi08a.c
        Drivers/BMI08X/Src/bmi08g.c
        Drivers/BMI08X/Src/bmi08xa.c
        Drivers/BMI08X/Src/bmi088_anymotiona.c
        Drivers/BMI08X/Src/bmi088_mma.c
        Drivers/BMM350/Src/bmm350.c
        Drivers/BMM350/Src/bmm350_oor.c
        Drivers/BNO055/Src/bno055.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined include paths
        Drivers/CMSIS_DSP/Include
        Drivers/BMM350/Inc
        Drivers/BMI08X/Inc
        Drivers/BNO055/Inc
        Drivers/Dynamixel/Inc
        Middlewares/HexapodMath/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined symbols
)

target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
        -Wall -Wextra -pedantic -Werror
)

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
        -u _printf_float
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
        stm32cubemx

        # Add user defined libraries
        dynamixel
        HexapodMath
        CMSISDSP

        # We need libm(ath) for some trig functions
        m
)

set(HEX_FILE ${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin)

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE} Building ${BIN_FILE}"
)
