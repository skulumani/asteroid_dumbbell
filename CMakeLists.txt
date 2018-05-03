cmake_minimum_required(VERSION 3.10)
project( asteroid_dumbell )

################################################################################
# GLOBAL SETUP
################################################################################
enable_language(CXX)
set(CMAKE_CXX_STANDARD 11)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# CGAL Setup
set(ENV{CGAL_DIR} /usr/local/lib/CGAL)
find_package(CGAL COMPONENTS)
if (NOT CGAL_FOUND)
   message(STATUS "This project requires the CGAL Library and will not compile")
   return()
endif()
# include helper file
include( ${CGAL_USE_FILE} )
message(STATUS "CGAL Lib: ${CGAL_LIBRARIES}")

# HDF5 Setup
set(ENV{HDF5_ROOT} /usr/local/hdf5)
find_package(HDF5 REQUIRED COMPONENTS C CXX)
if (NOT HDF5_FOUND)
    message(STATUS "This project requires HDF5 and will not compile")
    return()
else()
    message(STATUS "Found HDF5: ${HDF5_VERSION}")
endif()

if ( HDF5_IS_PARALLEL )
    message(STATUS "HDF5 parallel is installed")
else()
    message(STATUS "HDF5 parallel is NOT installed")
endif()
# message(STATUS "HDF5 Lib: ${HDF5_LIBRARIES}")
include_directories(${HDF5_INCLUDE_DIRS})
set(hdf5_libs hdf5 hdf5_cpp)

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


# just link executables against HDF5 target_link_libraries(target ${HDF5_CXX_LIBRARIES} ${HDF5_LIBRARIES})

# setup Python
#set(PYBIND11_PYTHON_VERSION 3.5 CACHE STRING "")
# include(${PROJECT_SOURCE_DIR}/FindPythonAnaconda.cmake)
# Pybind11 setup
find_package(PythonInterp REQUIRED)
find_package(PythonLibs ${PYTHON_VERSION_STRING} EXACT REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
link_libraries(${PYTHON_LIBRARIES})
link_directories(${PYTHON_LIBRARY_DIRS})

add_subdirectory(extern/pybind11)

# libigl setup
option(LIBIGL_USE_STATIC_LIBRARY     "Use libigl as static library" OFF)
option(LIBIGL_WITH_ANTTWEAKBAR       "Use AntTweakBar"    OFF)
option(LIBIGL_WITH_CGAL              "Use CGAL"           OFF)
option(LIBIGL_WITH_COMISO            "Use CoMiso"         OFF)
option(LIBIGL_WITH_CORK              "Use Cork"           OFF)
option(LIBIGL_WITH_EMBREE            "Use Embree"         OFF)
option(LIBIGL_WITH_LIM               "Use LIM"            OFF)
option(LIBIGL_WITH_MATLAB            "Use Matlab"         OFF)
option(LIBIGL_WITH_MOSEK             "Use MOSEK"          OFF)
option(LIBIGL_WITH_OPENGL            "Use OpenGL"         OFF)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"           OFF)
option(LIBIGL_WITH_OPENGL_GLFW_IMGUI "Use ImGui"          OFF)
option(LIBIGL_WITH_PNG               "Use PNG"            OFF)
option(LIBIGL_WITH_PYTHON            "Use Python"         OFF)
option(LIBIGL_WITH_TETGEN            "Use Tetgen"         OFF)
option(LIBIGL_WITH_TRIANGLE          "Use Triangle"       OFF)
option(LIBIGL_WITH_VIEWER            "Use OpenGL viewer"  OFF)
option(LIBIGL_WITH_XML               "Use XML"            OFF)

find_package(LIBIGL REQUIRED QUIET)


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
    ${PROJECT_SOURCE_DIR}/src/reconstruct.cpp
    ${PROJECT_SOURCE_DIR}/src/geodesic.cpp
    ${PROJECT_SOURCE_DIR}/src/polyhedron.cpp
    ${PROJECT_SOURCE_DIR}/src/lidar.cpp
    ${PROJECT_SOURCE_DIR}/src/controller.cpp
    ${PROJECT_SOURCE_DIR}/src/state.cpp
    ${PROJECT_SOURCE_DIR}/src/utilities.cpp
    ${PROJECT_SOURCE_DIR}/src/surface_mesher.cpp
    )
add_library(cgal_cpp SHARED ${cgal_src})

set(input_parser_src
    ${PROJECT_SOURCE_DIR}/src/input_parser.cpp
    )
add_library(input_parser_cpp SHARED ${input_parser_src})

set(eigen_hdf5_src 
    ${PROJECT_SOURCE_DIR}/src/hdf5_eigen.cpp
    ${PROJECT_SOURCE_DIR}/src/hdf5_file.cpp
    ${PROJECT_SOURCE_DIR}/src/hdf5_group.cpp
    ${PROJECT_SOURCE_DIR}/src/hdf5_dataset.cpp)
add_library(eigen_hdf5 SHARED ${eigen_hdf5_src})
target_link_libraries(eigen_hdf5 ${HDF5_CXX_LIBRARIES} ${HDF5_LIBRARIES})

# EXECUTABLES
# mesh loading
set(mesh_src src/mesh_main.cpp)
add_executable(mesh ${mesh_src})
target_link_libraries(mesh cgal_cpp input_parser_cpp)

# potential function
set(potential_src src/potential_main.cpp src/potential.cpp)
add_executable(potential ${potential_src})
target_link_libraries(potential igl::core)

# surface mesher
set( surface_mesher_src src/surface_mesher_main.cpp )
add_executable(surface_mesher ${surface_mesher_src})
target_link_libraries(surface_mesher input_parser_cpp cgal_cpp)

# incremental mesh reconstruction
set( reconstruct_mesh_src src/reconstruct_main.cpp )
add_executable(reconstruct_mesh ${reconstruct_mesh_src})
target_link_libraries(reconstruct_mesh cgal_cpp input_parser_cpp)

# exploration example 
set( explore_src src/explore_main.cpp)
add_executable(explore ${explore_src})
target_link_libraries(explore cgal_cpp input_parser_cpp eigen_hdf5
    ${HDF5_CXX_LIBRARIES} )

################################################################################
# PYTHON BINDING
################################################################################
pybind11_add_module(polyhedron_potential MODULE
src/potential.cpp src/potential_bindings.cpp)
target_link_libraries(polyhedron_potential PRIVATE igl::core)

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
    tests/cpp/test_controller.cpp
	tests/cpp/test_utilities.cpp
    tests/cpp/test_hdf5.cpp
    tests/cpp/test_libigl.cpp
    src/wavefront.cpp)

add_executable(test_all ${test_all_src})
target_link_libraries(test_all gtest 
    cgal_cpp 
    eigen_hdf5
    igl::core)
add_test(NAME test_all COMMAND test_all
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})


add_executable(test_hdf5 tests/cpp/test_hdf5.cpp tests/cpp/test_all.cpp)
target_link_libraries(test_hdf5 gtest
    eigen_hdf5)
add_test(NAME test_hdf5 COMMAND test_hdf5
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}) 

add_executable(test_libigl tests/cpp/test_libigl.cpp tests/cpp/test_all.cpp)
target_link_libraries(test_libigl gtest igl::core)
add_test(NAME test_libigl COMMAND test_libigl
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})

add_executable(test_potential tests/cpp/test_potential.cpp tests/cpp/test_all.cpp src/potential.cpp)
target_link_libraries(test_potential gtest igl::core)
add_test(NAME test_potential COMMAND test_potential,
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})

# this will install to /usr/local by default
# install(TARGETS read_obj DESTINATION bin)

# set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

 # also can use ${CMAKE_CURRENT_SOURCE_DIR} for current cmake location or ${CMAKE_SOURCE_DIR} for top leavel cmake
