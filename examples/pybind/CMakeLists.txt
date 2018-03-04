cmake_minimum_required(VERSION 2.8.12)
project(example)

FIND_PACKAGE(PythonInterp REQUIRED)
FIND_PACKAGE(PythonLibs ${PYTHON_VERSION_STRING} EXACT REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
link_libraries(${PYTHON_LIBRARIES})
link_directories(${PYTHON_LIBRARY_DIRS})

add_subdirectory(pybind11)

pybind11_add_module(example example.cpp)

