cmake_minimum_required(VERSION 3.5.1)

# Define project name
set(MAIN gameoflife)

find_package(SDL2 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2_ttf REQUIRED sdl2)

add_executable(${MAIN} ${MAIN}.cpp)

get_filename_component(FONT_FILE "Peepo.ttf" REALPATH)
target_compile_definitions(${MAIN} PRIVATE FONT_FILE="${FONT_FILE}")

include_directories(${SDL2_INCLUDE_DIRS})

target_link_libraries(${MAIN} linalg)
target_link_libraries(${MAIN} libcpphelpers)

target_link_libraries(${MAIN} ${SDL2_LIBRARIES})
target_link_libraries(${MAIN} SDL2_ttf)