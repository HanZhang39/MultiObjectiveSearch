project (Multi_objective_search)
cmake_minimum_required (VERSION 2.6)

set(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_COMPILER "/usr/bin/g++")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
SET (CMAKE_LINKER  "/usr/bin/ld")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -DOS_MAC")
SET(CMAKE_CXX_FLAGS_DEBUG  "${CMAKE_CXX_FLAGS} -g -Wall")

include_directories("inc")
include_directories("/opt/homebrew/opt/libomp/include")
include_directories("ndtree")

file(GLOB SOURCES "src/Solvers/*.cpp" "src/Utils/*.cpp" "src/CH/*.cpp")
# Find Boost
find_package( Boost REQUIRED COMPONENTS program_options log thread filesystem system)
include_directories( ${Boost_INCLUDE_DIRS} )



message(${CMAKE_BINARY_DIR})

add_executable(mohs src/mohs.cpp ${SOURCES} ${GCL_NDT_SOURCES})

target_link_libraries(mohs ${Boost_LIBRARIES})
