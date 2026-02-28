cmake_minimum_required(VERSION 3.22)


# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Feetech_Stm32)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})

# Shared Library (Used by BOTH)
set(DRIVER_SOURCES
    Lib/Sts_Servo/Src/sts_protocol.c
    Lib/Sts_Servo/Src/sts_servo.c
)

# STM32 Only 
set(FIRMWARE_SOURCES
    Core/Src/main.c
    Core/Src/stm32f4xx_it.c
    Core/Src/system_stm32f4xx.c
)

# PC Only 
set(TEST_SOURCES
    Tests/test_runner.c
    Tests/test_sts_protocol.c
    Tests/test_sts_utils.c
    Tests/Unity/unity.c
)

# Detect if we are using the ARM Toolchain or Native (Windows)
if(CMAKE_C_COMPILER_ID STREQUAL "GNU" AND CMAKE_C_COMPILER_TARGET STREQUAL "arm-none-eabi")
    set(IS_ARM_BUILD TRUE)
    enable_language(C ASM)
else()
    set(IS_ARM_BUILD FALSE)
    enable_language(C)
endif()



if(IS_ARM_BUILD)
    message("Configuring for STM32 (ARM)")
    add_executable(${CMAKE_PROJECT_NAME} ${FIRMWARE_SOURCES} ${DRIVER_SOURCES})
    add_subdirectory(cmake/stm32cubemx)
    target_link_libraries(${CMAKE_PROJECT_NAME} stm32cubemx)
    target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE 
        Core/Inc
        Lib/Sts_Servo/Inc  
        App/Inc
    )
else()
    message("Configuring for Native (Windows/Host)")
    add_executable(test_runner ${TEST_SOURCES} ${DRIVER_SOURCES})
    target_include_directories(test_runner PRIVATE 
        Lib/Sts_Servo/Inc
        Tests           # Added: Allows test files to find utils.h easily
        Tests/Unity
        App/Inc
    )
endif()





