cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Kawashiro_Frame_G474)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# ---------------------------------------------------------------------------
# CMSIS-DSP (provides "arm_math.h" used by the application)
# Downloaded from GitHub by default. For offline builds pass a local checkout:
#   cmake -B build -DCMSIS_DSP_SOURCE_DIR=/path/to/CMSIS-DSP
# ---------------------------------------------------------------------------
set(CMSIS_DSP_SOURCE_DIR "" CACHE PATH "Local path to a CMSIS-DSP source checkout (offline builds)")
# Keep downloaded dependencies outside the per-preset build directory so that
# Debug/Release (and future presets) share a single copy.
set(FETCHCONTENT_BASE_DIR "${CMAKE_SOURCE_DIR}/.deps" CACHE PATH "FetchContent download/checkout directory")
include(FetchContent)
if(CMSIS_DSP_SOURCE_DIR)
    set(FETCHCONTENT_SOURCE_DIR_cmsisdsp "${CMSIS_DSP_SOURCE_DIR}")
endif()
FetchContent_Declare(
    cmsisdsp
    GIT_REPOSITORY https://github.com/ARM-software/CMSIS-DSP.git
    GIT_TAG        v1.16.2
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(cmsisdsp)
# CMSIS-DSP relies on CMSIS Core headers (cmsis_compiler.h, core_cm4.h, ...);
# point it at the copy shipped with the STM32G4 HAL package.
target_include_directories(CMSISDSP PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Include)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Robot_Application
    Project/Robot_Application/RoboControl.c
    Project/Robot_Application/Chassis.c
    Project/Robot_Application/Communicate.c
    Project/Robot_Application/Gimbal.c
    Project/Robot_Application/INS.c
    Project/Robot_Application/Shoot.c
    Project/Robot_Application/Aim.c
    Project/Robot_Application/Key_LED.c
    Project/Robot_Application/ErrorHandle.c

    # Commnuicate_Drivers
    Project/Commnuicate_Drivers/CAN_Driver.c
    Project/Commnuicate_Drivers/USART_Driver.c
    Project/Commnuicate_Drivers/USB_Driver.c

    # Hardware_Drivers
    Project/Hardware_Drivers/Referee_Unpack.c
    Project/Hardware_Drivers/Remote_Control.c
    Project/Hardware_Drivers/SuperCap_Driver.c
    Project/Hardware_Drivers/Motor_Driver.c
    Project/Hardware_Drivers/Motor_DJI_Driver.c
    Project/Hardware_Drivers/Motor_DAMIAO_Driver.c

    # Algorithm_Drivers
    Project/Algorithm_Drivers/controller.c
    Project/Algorithm_Drivers/CRC8_CRC16.c
    Project/Algorithm_Drivers/Function.c
    Project/Algorithm_Drivers/kalman_filter.c
    Project/Algorithm_Drivers/PID.c
    Project/Algorithm_Drivers/Power_Limit.c
    Project/Algorithm_Drivers/QuaternionEKF.c
    Project/Algorithm_Drivers/user_lib.c
    Project/Algorithm_Drivers/Vofa.c
    Project/Algorithm_Drivers/Motor_Compensation.c

    # BSP
    Project/BSP/BMI088driver.c
    Project/BSP/BMI088Middleware.c
    Project/BSP/Buzzer.c
    Project/BSP/DWT.c
    Project/BSP/PWM.c
    Project/BSP/Key.c
    Project/BSP/LED.c
    Project/BSP/Flash.c

    # UI
    Project/UI/UI_Task.c
    Project/UI/ui_interface.c
    Project/UI/ui_g.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Project/Robot_Application
    Project/Commnuicate_Drivers
    Project/Hardware_Drivers
    Project/Algorithm_Drivers
    Project/BSP
    Project/UI
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    ARM_MATH_CM4
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    CMSISDSP

    # Add user defined libraries
)

# Generate .hex and .bin firmware images after linking (MDK also produces a .hex)
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary -S $<TARGET_FILE:${CMAKE_PROJECT_NAME}> $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${CMAKE_PROJECT_NAME}.bin
    COMMENT "Generating .hex and .bin firmware images"
)
