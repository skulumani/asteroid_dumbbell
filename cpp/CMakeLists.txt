cmake_minimum_required(VERSION 3.10)
project( wavefront )

set(CMAKE_CXX_STANDARD 11)
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

include_directories(${PROJECT_SOURCE_DIR}/include)
# create a obj file reader executable
set(read_obj_src src/main.cpp src/read_obj.cpp src/input_parser.cpp)
add_executable(read_obj ${read_obj_src})
target_link_libraries(read_obj gtest_main)
# add_test(NAME example_test COMMAND example)

## BUILD POLYHEDRON EXECUTABLE
set(build_poly_src src/build_poly.cpp src/read_obj.cpp src/input_parser.cpp)
add_executable(build_poly ${build_poly_src})
target_link_libraries(build_poly gtest_main)

# this will install to /usr/local by default
# install(TARGETS read_obj DESTINATION bin)

# set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

 set_target_properties( read_obj
   PROPERTIES
   ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
   LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
   RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
 )

 # also can use ${CMAKE_CURRENT_SOURCE_DIR} for current cmake location or ${CMAKE_SOURCE_DIR} for top leavel cmake
