cmake_minimum_required(VERSION 3.10)
project(boost_example)

enable_language(CXX)
set(CMAKE_CXX_STANDARD 11)

# Boost and its components
# set(Boost_USE_STATIC_LIBS OFF) 
# set(Boost_USE_MULTITHREADED ON)  
# set(Boost_USE_STATIC_RUNTIME OFF) 
find_package( Boost 1.66 COMPONENTS log REQUIRED)
if ( NOT Boost_FOUND )
  message(STATUS "This project requires the Boost library, and will not be compiled.")
  return()  
endif()

find_package(Threads)

add_definitions(-DBOOST_LOG_DYN_LINK)
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

add_executable(boost_test src/boost_test.cpp)
target_link_libraries(boost_test LINK_PUBLIC ${Boost_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

add_executable(boost_log src/boost_log.cpp)
target_link_libraries(boost_log LINK_PUBLIC ${Boost_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})


