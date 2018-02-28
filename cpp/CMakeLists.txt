cmake_minimum_required(VERSION 3.10)
project( wavefront )

#include_directories(${PROJECT_SOURCE_DIR}/include)
# create a obj file reader executable
set(read_obj_src src/read_obj.cpp )
add_executable(read_obj ${read_obj_src})
install(TARGETS read_obj DESTINATION bin)

