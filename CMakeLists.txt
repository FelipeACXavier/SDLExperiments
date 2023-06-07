cmake_minimum_required(VERSION 3.5.1)

# Define project name
project(SDLExperiment VERSION 1.0)

add_compile_options(-O2)

# Add global libraries
add_subdirectory(linalg)
add_subdirectory(CppHelpers)

# Add each experiment
if( ${CMAKE_SYSTEM_NAME} MATCHES "Emscripten")
  add_subdirectory(web)
else()
  add_subdirectory(game_of_life)
  add_subdirectory(wave_generation)
  add_subdirectory(boids)
endif()