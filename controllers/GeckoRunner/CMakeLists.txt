cmake_minimum_required(VERSION 3.0)

# Debug
set( CMAKE_BUILD_TYPE "Debug" )
set( CMAKE_CXX_FLAGS_DEBUG "-o0 -Wall -g -ggdb" )

# Setup the project.
# Its name is defined to be the controller directory name.
get_filename_component(PROJECT ${CMAKE_SOURCE_DIR} NAME)
project(${PROJECT})

# Get C or C++ sources in the current directory (only).
file(GLOB C_SOURCES ./src/*.c)
file(GLOB CPP_SOURCES ./src/*.cpp)
set(SOURCES ${C_SOURCES} ${CPP_SOURCES})

# Link with the Webots controller library.
link_directories($ENV{WEBOTS_HOME}/lib/controller)
set (LIBRARIES ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX} ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})
include_directories(/Applications/Webots.app/include/controller/c /Applications/Webots.app/include/controller/cpp ./include /usr/local/include/eigen3)

# Setup the target executable.
add_executable(${PROJECT} ${SOURCES})
target_link_libraries(${PROJECT} ${LIBRARIES})

# Copy the target executable at the right location.
add_custom_command(TARGET ${PROJECT} POST_BUILD COMMAND ${CMAKE_COMMAND} -E
  copy ${CMAKE_BINARY_DIR}/${PROJECT} ${CMAKE_SOURCE_DIR})
