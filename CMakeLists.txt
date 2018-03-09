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

# create a obj file reader executable
set(read_obj_src src/read_obj_main.cpp src/wavefront.cpp src/input_parser.cpp)
add_executable(read_obj ${read_obj_src})
target_link_libraries(read_obj gtest_main)


# BUILD POLYHEDRON EXECUTABLE
set(build_poly_src src/build_poly_main.cpp src/wavefront.cpp src/input_parser.cpp src/polyhedron.cpp)
add_executable(build_poly ${build_poly_src})
add_to_cached_list(CGAL_EXECUTABLE_TARGETS build_poly)
target_link_libraries(build_poly gtest_main ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES})

# mesh loading
set(mesh_src src/mesh.cpp src/mesh_main.cpp src/loader.cpp)
add_executable(mesh ${mesh_src})
################################################################################
# PYTHON BINDING
################################################################################
pybind11_add_module(cgal_cpp MODULE 
    src/bindings.cpp
    src/polyhedron.cpp)

################################################################################
# Testing example
################################################################################
ENABLE_TESTING()

set(test_all_src 
    tests/cpp/test_all.cpp
    tests/cpp/test_polyhedron.cpp
    tests/cpp/test_gtest.cpp
    tests/cpp/test_wavefront.cpp
    src/wavefront.cpp
    src/polyhedron.cpp)

add_executable(test_all ${test_all_src})
target_link_libraries(test_all gtest)
add_test(NAME test_all COMMAND test_all
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})

add_executable(test_wavefront src/wavefront.cpp tests/cpp/test_wavefront.cpp tests/cpp/test_all.cpp)
target_link_libraries(test_wavefront gtest)
add_test(NAME test_wavefront COMMAND test_wavefront
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})

add_executable(test_polyhedron src/wavefront.cpp src/polyhedron.cpp tests/cpp/test_polyhedron.cpp tests/cpp/test_all.cpp)
target_link_libraries(test_polyhedron gtest)
add_test(NAME test_polyhedron COMMAND test_polyhedron
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
# this will install to /usr/local by default
# install(TARGETS read_obj DESTINATION bin)

# set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

 # also can use ${CMAKE_CURRENT_SOURCE_DIR} for current cmake location or ${CMAKE_SOURCE_DIR} for top leavel cmake
