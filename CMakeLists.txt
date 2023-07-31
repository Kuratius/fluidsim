cmake_minimum_required(VERSION 3.22)
set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
add_subdirectory(dependencies/minifb)

project(fluidsim)


add_executable(fluidsim src/fluidsim.c)
# Link MiniFB to your project:
target_link_libraries(fluidsim
    minifb
)

