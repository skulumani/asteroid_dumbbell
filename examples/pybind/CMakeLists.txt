cmake_minimum_required(VERSION 2.8.12)
project(example)

set(CMAKE_CXX_STANDARD 11)

FIND_PACKAGE(PythonInterp REQUIRED)
FIND_PACKAGE(PythonLibs ${PYTHON_VERSION_STRING} EXACT REQUIRED)

include_directories(${PYTHON_INCLUDE_DIRS})
link_libraries(${PYTHON_LIBRARIES})
link_directories(${PYTHON_LIBRARY_DIRS})
    
set(SOURCE_DIR "src")
include_directories(${SOURCE_DIR})

add_subdirectory(lib/pybind11)

# add_executable(pointer src/pointer.cpp)
pybind11_add_module(shared_pointer MODULE src/pointer.cpp)

# Can build a module or a shared library 
pybind11_add_module(example MODULE ${SOURCE_DIR}/example.cpp)
pybind11_add_module(python_example MODULE 
    ${SOURCE_DIR}/math.cpp 
    ${SOURCE_DIR}/bindings.cpp
    ${SOURCE_DIR}/eigen.cpp)

# add_executable(template src/template.cpp)
pybind11_add_module(template MODULE ${SOURCE_DIR}/template.cpp)



