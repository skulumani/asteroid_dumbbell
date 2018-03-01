cmake_minimum_required(VERSION 3.10)
project( wavefront )

set(CMAKE_CXX_STANDARD 11)

include_directories(${PROJECT_SOURCE_DIR}/include)
# create a obj file reader executable
set(read_obj_src src/read_obj.cpp src/input_parser.cpp)
add_executable(read_obj ${read_obj_src})

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
