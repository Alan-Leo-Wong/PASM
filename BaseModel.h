#pragma once
#include <Eigen/Dense>
//#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
//#include <CGAL/Surface_mesh.h>
//#include <CGAL/Polyhedron_3.h>
//#include <CGAL/point_generators_3.h>
//#include <CGAL/Search_traits_3.h>
//#include <CGAL/Triangle_mesh_point_locator_3.h>

//typedef CGAL::Exact_predicates_exact_constructions_kernel K;
//typedef K::Point_3 Point_3;
//typedef K::Triangle_3 Triangle_3;
//typedef CGAL::Surface_mesh<Point_3> Mesh;
//typedef CGAL::Polyhedron_3<K> Polyhedron;
//typedef boost::graph_traits<Mesh>::face_descriptor Face_descriptor;

struct BaseModel
{
public:
	//Mesh mesh;
	Eigen::MatrixXd V;
	Eigen::MatrixXi F;

public:
	BaseModel(const std::string& filename) { readMesh(filename); }

	void readMesh(const std::string& filename);
};

Eigen::Vector3d point2MeshProjection(const Eigen::Vector3d& point, const BaseModel& model);