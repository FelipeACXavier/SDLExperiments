cmake_minimum_required(VERSION 3.5.1)

# Define project name
set(BOIDS boids)

find_package(SDL2 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2_ttf REQUIRED sdl2)

add_executable(${BOIDS} ${BOIDS}.cpp)

get_filename_component(FONT_FILE "Peepo.ttf" REALPATH)
target_compile_definitions(${BOIDS} PRIVATE FONT_FILE="${FONT_FILE}")

include_directories(${SDL2_INCLUDE_DIRS})

target_link_libraries(${BOIDS} linalg)
target_link_libraries(${BOIDS} libcpphelpers)

target_link_libraries(${BOIDS} ${SDL2_LIBRARIES})
target_link_libraries(${BOIDS} SDL2_ttf)