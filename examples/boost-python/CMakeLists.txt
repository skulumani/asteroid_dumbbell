cmake_minimum_required(VERSION 3.10.2)
project(boost_python)

# Find and link all the python stuff
FIND_PACKAGE(PythonInterp REQUIRED)
FIND_PACKAGE(PythonLibs ${PYTHON_VERSION_STRING} EXACT REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
link_libraries(${PYTHON_LIBRARIES})
link_directories(${PYTHON_LIBRARY_DIRS})

# Now we find the required Boost libraries using the canonical name
if (${PYTHON_VERSION_MAJOR} EQUAL 3)
    FIND_PACKAGE(Boost 1.66 REQUIRED COMPONENTS python3 numpy3)
else()
    FIND_PACKAGE(Boost 1.66 REQUIRED COMPONENTS python numpy)
endif()
link_libraries(${Boost_LIBRARIES})
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

ENABLE_TESTING()

ADD_SUBDIRECTORY(01_hello_world)
add_subdirectory(02_exposing_classes)
add_subdirectory(03_constructors)
add_subdirectory(04_numpy_example)
add_subdirectory(05_class_members)
# each subdiretory that you want to build into a python shared library should have the following
# this will create a so file for import into python
# PYTHON_ADD_MODULE(test_numpy test_numpy.cpp)

# this is another option
# add_library(test_numpy SHARED test_numpy.cpp)
# set_target_properties(test_numpy PROPERTIES PREFIX "" SUFFIX ".so")
# set_target_properties(test_numpy PROPERTIES DEFINE_SYMBOL "BOOST_ALL_NO_LIB")
# target_link_libraries(test_numpy ${PYTHON_LIBRARIES} ${Boost_LIBRARIES})


