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

# Eigen
find_package(Eigen3 REQUIRED)
include(${EIGEN3_USE_FILE})

# Boost and its components
find_package( Boost REQUIRED )

if ( NOT Boost_FOUND )

  message(STATUS "This project requires the Boost library, and will not be compiled.")

  return()  

endif()

# include for local directory

# include for local package

# ALL THE EXECUTABLES

add_executable( points_and_segments  src/points_and_segments.cpp )
add_to_cached_list( CGAL_EXECUTABLE_TARGETS points_and_segments )
target_link_libraries(points_and_segments   ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} )

add_executable( surprising src/surprising.cpp )
add_to_cached_list( CGAL_EXECUTABLE_TARGETS surprising )
target_link_libraries(surprising   ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} )

add_executable( exact src/exact.cpp )
add_to_cached_list( CGAL_EXECUTABLE_TARGETS exact )
target_link_libraries(exact   ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} )

add_executable( points_and_vectors src/points_and_vectors.cpp )
add_to_cached_list( CGAL_EXECUTABLE_TARGETS points_and_vectors)
target_link_libraries(points_and_vectors  ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} )

add_executable( hds_prog_default src/hds_prog_default.cpp )
add_to_cached_list( CGAL_EXECUTABLE_TARGETS hds_prog_default )
target_link_libraries(hds_prog_default   ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} )

add_executable( hds_prog_graph src/hds_prog_graph.cpp )
add_to_cached_list( CGAL_EXECUTABLE_TARGETS hds_prog_graph )
target_link_libraries(hds_prog_graph   ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} )

add_executable( polyhedron_prog_tetra src/polyhedron_prog_tetra.cpp )
add_to_cached_list( CGAL_EXECUTABLE_TARGETS polyhedron_prog_tetra )
target_link_libraries(polyhedron_prog_tetra   ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} )

add_executable( polyhedron_prog_incr_builder src/polyhedron_prog_incr_builder.cpp )
add_to_cached_list( CGAL_EXECUTABLE_TARGETS polyhedron_prog_incr_builder )
target_link_libraries(polyhedron_prog_incr_builder   ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} )

add_executable( AABB_polyhedron_facet_intersection_example src/AABB_polyhedron_facet_intersection_example.cpp )
add_to_cached_list( CGAL_EXECUTABLE_TARGETS AABB_polyhedron_facet_intersection_example )
target_link_libraries(AABB_polyhedron_facet_intersection_example ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES})

add_executable( AABB_triangle_3_example src/AABB_triangle_3_example.cpp )
add_to_cached_list( CGAL_EXECUTABLE_TARGETS AABB_triangle_3_example )
target_link_libraries(AABB_triangle_3_example ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES})

add_executable( searching_surface_mesh_vertices src/searching_surface_mesh_vertices.cpp )
add_to_cached_list( CGAL_EXECUTABLE_TARGETS searching_surface_mesh_vertices )
target_link_libraries(searching_surface_mesh_vertices ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES})

add_executable( mesh_implicit_function src/mesh_implicit_function.cpp )
add_to_cached_list( CGAL_EXECUTABLE_TARGETS mesh_implicit_function )
target_link_libraries(mesh_implicit_function ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES})

add_executable(refine_and_fair src/refine_fair_example.cpp)
add_to_cached_list(CGAL_EXECUTABLE_TARGETS refine_and_fair)
target_link_libraries(refine_and_fair ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES})

add_executable(isotropic_refine src/isotropic_remeshing_example.cpp)
add_to_cached_list(CGAL_EXECUTABLE_TARGETS isotropic_refine)
target_link_libraries(isotropic_refine ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES})

add_executable(isotropic_refine_patch src/isotropic_remeshing_of_patch_example.cpp)
add_to_cached_list(CGAL_EXECUTABLE_TARGETS isotropic_refine_patch)
target_link_libraries(isotropic_refine_patch ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES})
