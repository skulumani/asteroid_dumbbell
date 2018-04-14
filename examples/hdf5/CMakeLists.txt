cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(hdf5-example)

set(ENV{HDF5_ROOT} /usr/local)

enable_language(CXX)

find_package(HDF5 REQUIRED COMPONENTS C CXX)
if (NOT HDF5_FOUND)
    message(STATUS "This project requires HDF5 and will not compile")
    return()
endif()
set(hdf5_libs hdf5 hdf5_cpp)

include_directories(${HDF5_INCLUDE_DIRS})

add_executable(writedata src/writedata.cpp)
target_link_libraries(writedata ${HDF5_CXX_LIBRARIES} ${HDF5_LIBRARIES})

add_executable(simple src/hdf5_simple.cpp)
target_link_libraries(simple ${HDF5_CXX_LIBRARIES} ${HDF5_LIBRARIES})
