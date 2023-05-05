cmake_minimum_required(VERSION 3.5.1)

# Define project name
set(EXEC web)

add_executable(${EXEC} ${EXEC}.cpp)
include_directories(${SDL2_INCLUDE_DIRS})
include_directories(${SDL2TTF_INCLUDE_DIRS})
include_directories(${SDL2IMAGE_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})

# target_link_libraries(${EXEC} linalg)
target_link_libraries(${EXEC} libcpphelpers)
target_link_libraries(${EXEC} ${SDL2_LIBRARIES})

file(
  COPY
    ${CMAKE_CURRENT_SOURCE_DIR}/res
    ${CMAKE_CURRENT_SOURCE_DIR}/index.html
  DESTINATION
    ${CMAKE_BINARY_DIR}/${EXEC}
)

set_target_properties(${EXEC}
  PROPERTIES LINK_FLAGS
    "-s WASM=1 \
    -s USE_SDL=2 \
    -s USE_SDL_TTF=2 \
    -s USE_SDL_IMAGE=2 \
    -s SDL2_IMAGE_FORMATS='[\"png\"]' \
    --use-preload-plugins \
    --preload-file res \
    -s EXPORTED_FUNCTIONS='[_main]'"
)