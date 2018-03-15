#ifndef CGAL_H
#define CGAL_H

#include "cgal_types.hpp"
#include "mesh.hpp"

#include <memory>

void distance_to_polyhedron(Eigen::Vector3d& pt, std::shared_ptr<MeshData> mesh);

#endif
