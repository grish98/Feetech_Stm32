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
set(CMAKE_PROJECT_NAME Feetech_Stm32)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})

# Detect if we are using the ARM Toolchain or Native (Windows)
if(CMAKE_C_COMPILER_ID STREQUAL "GNU" AND CMAKE_C_COMPILER_TARGET STREQUAL "arm-none-eabi")
    set(IS_ARM_BUILD TRUE)
    enable_language(C ASM)
else()
    set(IS_ARM_BUILD FALSE)
    enable_language(C)
endif()


# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
    Tests/test_runner.c
    Tests/test_sts_protocol.c
    Tests/Unity/unity.c
    
    Lib/Sts_Servo/Src/sts_protocol.c
    Lib/Sts_Servo/Src/sts_servo.c

)


if(IS_ARM_BUILD)
    message("Configuring for STM32 (ARM)")
    add_subdirectory(cmake/stm32cubemx)
    target_link_libraries(${CMAKE_PROJECT_NAME} stm32cubemx)
    target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE Core/Inc)
else()
    message("Configuring for Native (Windows/Host)")

endif()


target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE            
    Lib/Sts_Servo/Inc    
    Tests/Unity 
    App/Inc         
)



