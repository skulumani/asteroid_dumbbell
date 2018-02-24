cmake_minimum_required(VERSION 2.6)
project(boost_python)

FIND_PACKAGE(PythonInterp)
FIND_PACKAGE(PythonLibs)

# Now we find the required Boost libraries using the canonical name
if (${PYTHON_VERSION_MAJOR} EQUAL 3)
    FIND_PACKAGE(Boost REQUIRED COMPONENTS python3)
else()
    FIND_PACKAGE(Boost REQUIRED COMPONENTS python)
endif()

ENABLE_TESTING()
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS})
LINK_LIBRARIES(${Boost_LIBRARIES} ${PYTHON_LIBRARIES})

ADD_SUBDIRECTORY(01_hello_world)
add_subdirectory(02_exposing_classes)
add_subdirectory(03_constructors)


