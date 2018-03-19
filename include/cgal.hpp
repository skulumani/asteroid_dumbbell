#ifndef CGAL_H
#define CGAL_H

#include "cgal_types.hpp"
#include "mesh.hpp"

#include <memory>

void distance_to_polyhedron(Eigen::Vector3d& pt, std::shared_ptr<MeshData> mesh);

class RayCaster {
    public:
        RayCaster(std::shared_ptr<MeshData> mesh_in);

        // cast ray function
        void castray(Eigen::Vector3d& psource, Eigen::Vector3d& ptarget);
        // cast many rays function
        void update_mesh(std::shared_ptr<MeshData> mesh_in);
    private:
        // needs the mesh to operate on
        std::shared_ptr<MeshData> mesh;
};

#endif
