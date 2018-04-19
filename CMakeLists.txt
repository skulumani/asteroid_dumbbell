cmake_minimum_required(VERSION 3.10)
project( wavefront )

################################################################################
# GLOBAL SETUP
################################################################################
set(CMAKE_CXX_STANDARD 11)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

# CGAL Setup
find_package(CGAL QUIET COMPONENTS)
if (NOT CGAL_FOUND)
   message(STATUS "This project requires the CGAL Library and will not compile")
   return()
endif()
# include helper file
include( ${CGAL_USE_FILE} )

# Eigen setup
find_package(Eigen3 REQUIRED)
if (NOT Eigen3_FOUND)
    message(STATUS "This project requires Eigen3 and will not be compiled.")
    return()
endif()
include(${EIGEN3_USE_FILE})

# Boost and its components
find_package( Boost REQUIRED )
if ( NOT Boost_FOUND )
  message(STATUS "This project requires the Boost library, and will not be compiled.")
  return()  
endif()

find_package(PythonInterp REQUIRED)
find_package(PythonLibs ${PYTHON_VERSION_STRING} EXACT REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
link_libraries(${PYTHON_LIBRARIES})
link_directories(${PYTHON_LIBRARY_DIRS})

add_subdirectory(extern/pybind11)
################################################################################
# GOOGLE TESTING
################################################################################
# Download and unpack googletest at configure time
configure_file(CMakeLists.txt.in googletest-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download )
if(result)
  message(FATAL_ERROR "CMake step for googletest failed: ${result}")
endif()
execute_process(COMMAND ${CMAKE_COMMAND} --build .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download )
if(result)
  message(FATAL_ERROR "Build step for googletest failed: ${result}")
endif()

# Prevent overriding the parent project's compiler/linker
# settings on Windows
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# Add googletest directly to our build. This defines
# the gtest and gtest_main targets.
add_subdirectory(${CMAKE_BINARY_DIR}/googletest-src
                 ${CMAKE_BINARY_DIR}/googletest-build
                 EXCLUDE_FROM_ALL)

# The gtest/gtest_main targets carry header search path
# dependencies automatically when using CMake 2.8.11 or
# later. Otherwise we have to add them here ourselves.
if (CMAKE_VERSION VERSION_LESS 2.8.11)
  include_directories("${gtest_SOURCE_DIR}/include")
endif()

################################################################################
# BUILD TARGETS
################################################################################
include_directories(${PROJECT_SOURCE_DIR}/include)

# LIBRARIES
# create some shared libraries for things
set(cgal_src
    ${PROJECT_SOURCE_DIR}/src/mesh.cpp
    ${PROJECT_SOURCE_DIR}/src/polyhedron.cpp
    ${PROJECT_SOURCE_DIR}/src/loader.cpp
    ${PROJECT_SOURCE_DIR}/src/stats.cpp
    ${PROJECT_SOURCE_DIR}/src/cgal.cpp
    )
add_library(cgal_cpp SHARED ${cgal_src})

set(input_parser_src
    ${PROJECT_SOURCE_DIR}/src/input_parser.cpp
    )
add_library(input_parser_cpp SHARED ${input_parser_src})

set(potential_src 
    ${PROJECT_SOURCE_DIR}/src/potential.cpp
    )
add_library(potential_cpp SHARED ${potential_src})

set(surface_mesher_src
    ${PROJECT_SOURCE_DIR}/src/surface_mesher.cpp
    )
add_library(surface_mesher_cpp SHARED ${surface_mesher_src})

set(polyhedron_src
    ${PROJECT_SOURCE_DIR}/src/reconstruct.cpp
    ${PROJECT_SOURCE_DIR}/src/geodesic.cpp
    ${PROJECT_SOURCE_DIR}/src/polyhedron.cpp
    ${PROJECT_SOURCE_DIR}/src/lidar.cpp
    ${PROJECT_SOURCE_DIR}/src/controller.cpp)
add_library(polyhedron_cpp SHARED ${polyhedron_src})

# EXECUTABLES
# mesh loading
set(mesh_src src/mesh_main.cpp)
add_executable(mesh ${mesh_src})
target_link_libraries(mesh cgal_cpp input_parser_cpp)

# potential function
set(potential_src src/potential_main.cpp)
add_executable(potential ${potential_src})
target_link_libraries(potential potential_cpp)

# surface mesher
set( surface_mesher_src src/surface_mesher_main.cpp )
add_executable(surface_mesher ${surface_mesher_src})
target_link_libraries(surface_mesher input_parser_cpp surface_mesher_cpp)

# incremental mesh reconstruction
set( reconstruct_mesh_src src/reconstruct_main.cpp )
add_executable(reconstruct_mesh ${reconstruct_mesh_src})
target_link_libraries(reconstruct_mesh polyhedron_cpp cgal_cpp input_parser_cpp)

# exploration example 
set( explore_src src/explore_main.cpp)
add_executable(explore ${explore_src})
target_link_libraries(explore cgal_cpp polyhedron_cpp input_parser_cpp surface_mesher_cpp)
################################################################################
# PYTHON BINDING
################################################################################
pybind11_add_module(polyhedron_potential MODULE
    src/potential.cpp src/potential_bindings.cpp)

pybind11_add_module(surface_mesh MODULE
    src/surface_mesher.cpp src/surface_mesh_bindings.cpp)

pybind11_add_module(mesh_data MODULE
    src/mesh.cpp src/polyhedron.cpp src/mesh_bindings.cpp)

pybind11_add_module(cgal MODULE
    src/cgal.cpp src/cgal_bindings.cpp src/mesh.cpp)

pybind11_add_module(reconstruct MODULE
    src/reconstruct.cpp src/reconstruct_bindings.cpp src/mesh.cpp
    src/polyhedron.cpp src/geodesic.cpp)

pybind11_add_module(geodesic MODULE
    src/geodesic.cpp src/geodesic_bindings.cpp)

################################################################################
# Testing example
################################################################################
ENABLE_TESTING()

set(test_all_src 
    tests/cpp/test_all.cpp
    tests/cpp/test_gtest.cpp
    tests/cpp/test_wavefront.cpp
    tests/cpp/test_surface_mesher.cpp
    tests/cpp/test_polyhedron.cpp
    tests/cpp/test_loader.cpp
    tests/cpp/test_mesh.cpp
    tests/cpp/test_reconstruct.cpp
    tests/cpp/test_geodesic.cpp
    tests/cpp/test_cgal.cpp
	tests/cpp/test_lidar.cpp
    src/wavefront.cpp)

add_executable(test_all ${test_all_src})
target_link_libraries(test_all gtest cgal_cpp potential_cpp surface_mesher_cpp 
    polyhedron_cpp)
add_test(NAME test_all COMMAND test_all
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})


# this will install to /usr/local by default
# install(TARGETS read_obj DESTINATION bin)

# set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

 # also can use ${CMAKE_CURRENT_SOURCE_DIR} for current cmake location or ${CMAKE_SOURCE_DIR} for top leavel cmake
