/**
    A bunch of useful functions for operating on the MeshData class

    @author Shankar Kulumani
    @version Date
*/
#ifndef STATS_H
#define STATS_H

#include <Eigen/Dense>

#include <memory>

class MeshData;
class Asteroid;
class MeshParam;
class ReconstructMesh;

namespace PolyVolume {
    double volume(const Eigen::Ref<const Eigen::MatrixXd> &v, 
                             const Eigen::Ref<const Eigen::MatrixXi> &f);
    double volume(std::shared_ptr<const MeshData> meshdata_ptr);
    double volume(std::shared_ptr<const Asteroid> ast_ptr);
    double volume(std::shared_ptr<const MeshParam> meshparam_ptr);
    double volume(std::shared_ptr<const ReconstructMesh> rmesh_ptr);

}

void print_polyhedron_vertices(std::shared_ptr<MeshData> mesh);

void print_polyhedron_faces(std::shared_ptr<MeshData> mesh);

void print_polyhedron_stats(std::shared_ptr<MeshData> mesh);

void print_surface_mesh_vertices(std::shared_ptr<MeshData> mesh);

void surface_mesh_stats(std::shared_ptr<MeshData> mesh);
#endif
