cmake_minimum_required(VERSION 3.10)
project(pendulum_kinematics)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Link raylib directly from installation
set(RAYLIB_PATH /usr/local)
link_directories(${RAYLIB_PATH}/lib)
include_directories(${RAYLIB_PATH}/include)

# Collect all source files
file(GLOB_RECURSE SOURCES
    "src/**/*.cpp"
)

# Ensure main.cpp is included
list(APPEND SOURCES "${CMAKE_SOURCE_DIR}/src/main.cpp")

# Create executable
add_executable(pendulum_sim ${SOURCES})

# Include directories
target_include_directories(pendulum_sim PRIVATE "${CMAKE_SOURCE_DIR}/inlcude")

# Link libraries
target_link_libraries(pendulum_sim raylib)

# Set compiler flags for raylib
if(UNIX)
    target_link_libraries(pendulum_sim m pthread X11)
endif()
