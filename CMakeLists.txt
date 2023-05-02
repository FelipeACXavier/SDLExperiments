cmake_minimum_required(VERSION 3.5.1)

# Define project name
project(SDLExperiment VERSION 1.0)

add_compile_options(-O3)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add global libraries
add_subdirectory(linalg)
add_subdirectory(CppHelpers)

# Add each experiment
add_subdirectory(wave_generation)
add_subdirectory(boids)