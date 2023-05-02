cmake_minimum_required(VERSION 3.5.1)

# Define project name
set(BOIDS boids)

find_package(SDL2 REQUIRED)

add_executable(${BOIDS} ${BOIDS}.cpp)

include_directories(${SDL2_INCLUDE_DIRS})

target_link_libraries(${BOIDS} linalg)
target_link_libraries(${BOIDS} libcpphelpers)

target_link_libraries(${BOIDS} ${SDL2_LIBRARIES})