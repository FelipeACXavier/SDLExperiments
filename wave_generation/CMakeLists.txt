cmake_minimum_required(VERSION 3.5.1)

# Define project name
set(WAVE_GENERATION wave_generation)

find_package(SDL2 REQUIRED)

add_executable(${WAVE_GENERATION} ${WAVE_GENERATION}.cpp)

include_directories(${SDL2_INCLUDE_DIRS})

target_link_libraries(${WAVE_GENERATION} linalg)
target_link_libraries(${WAVE_GENERATION} ${SDL2_LIBRARIES})