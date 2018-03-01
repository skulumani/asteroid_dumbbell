# Created by the script cgal_create_CMakeLists
# This is the CMake script for compiling a set of CGAL applications.

project( cgal_testing )


cmake_minimum_required(VERSION 2.8.11)

# CGAL and its components
find_package( CGAL QUIET COMPONENTS  )

if ( NOT CGAL_FOUND )

  message(STATUS "This project requires the CGAL library, and will not be compiled.")
  return()  

endif()

# include helper file
include( ${CGAL_USE_FILE} )


# Boost and its components
find_package( Boost REQUIRED )

if ( NOT Boost_FOUND )

  message(STATUS "This project requires the Boost library, and will not be compiled.")

  return()  

endif()

# include for local directory

# include for local package

# ALL THE EXECUTABLES

add_executable( points_and_segments  points_and_segments.cpp )
add_to_cached_list( CGAL_EXECUTABLE_TARGETS points_and_segments )
target_link_libraries(points_and_segments   ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} )

add_executable( surprising surprising.cpp )
add_to_cached_list( CGAL_EXECUTABLE_TARGETS surprising )
target_link_libraries(surprising   ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} )

add_executable( exact exact.cpp )
add_to_cached_list( CGAL_EXECUTABLE_TARGETS exact )
target_link_libraries(exact   ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} )

add_executable( points_and_vectors points_and_vectors.cpp )
add_to_cached_list( CGAL_EXECUTABLE_TARGETS points_and_vectors)
target_link_libraries(points_and_vectors  ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} )

add_executable( hds_prog_default hds_prog_default.cpp )
add_to_cached_list( CGAL_EXECUTABLE_TARGETS hds_prog_default )
target_link_libraries(hds_prog_default   ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} )

add_executable( hds_prog_graph hds_prog_graph.cpp )
add_to_cached_list( CGAL_EXECUTABLE_TARGETS hds_prog_graph )
target_link_libraries(hds_prog_graph   ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} )

add_executable( polyhedron_prog_tetra polyhedron_prog_tetra.cpp )
add_to_cached_list( CGAL_EXECUTABLE_TARGETS polyhedron_prog_tetra )
target_link_libraries(polyhedron_prog_tetra   ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} )

add_executable( polyhedron_prog_incr_builder polyhedron_prog_incr_builder.cpp )
add_to_cached_list( CGAL_EXECUTABLE_TARGETS polyhedron_prog_incr_builder )
target_link_libraries(polyhedron_prog_incr_builder   ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} )
