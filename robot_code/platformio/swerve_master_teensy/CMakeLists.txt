cmake_minimum_required(VERSION 3.16)
project(SwerveSimulation)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add compile definitions
add_definitions(-DHAL_IMPLEMENTATION=HAL_SIM)

# Include directories
include_directories(include)
include_directories(src)

# Find required packages
find_package(Threads REQUIRED)

# Source files for simulation
set(SIMULATION_SOURCES
    src/hal/HALFactory.cpp
    src/Kinematics.cpp
    src/Planner.cpp
    src/Performance.cpp
    src/Swerve.cpp
    src/shared/utils.cpp
    src/shared/Channel.cpp
    src/shared/LowPassFilter.cpp
    src/penny/Pads.cpp
    src/penny/Lights.cpp
    # Add other source files as needed
)

# Test executables
add_executable(simple_test src/simple_test.cpp)
target_link_libraries(simple_test Threads::Threads)

add_executable(test_native src/test_native.cpp)
target_link_libraries(test_native Threads::Threads)

# Main simulation executable (when robot_main.cpp is ready)
add_executable(robot_simulation 
    src/robot_main.cpp
    ${SIMULATION_SOURCES}
)
target_link_libraries(robot_simulation Threads::Threads)

# Set output directory
set_target_properties(simple_test test_native robot_simulation
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# Compiler-specific options
if(MSVC)
    target_compile_options(simple_test PRIVATE /W4)
    target_compile_options(test_native PRIVATE /W4)
    target_compile_options(robot_simulation PRIVATE /W4)
else()
    target_compile_options(simple_test PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(test_native PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(robot_simulation PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Print configuration info
message(STATUS "Building for simulation with HAL_SIM")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER_ID}")
