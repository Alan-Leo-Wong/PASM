#include "BaseModel.h"
#include <vector>
#include <fstream>
#include <iostream>
//#include <CGAL/property_map.h>
//#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
//namespace PMP = CGAL::Polygon_mesh_processing;
#include <nanoflann.hpp>
#include <igl/read_triangle_mesh.h>
#include <igl/point_mesh_squared_distance.h>
#include <igl/barycentric_coordinates.h>
#include <igl/barycentric_interpolation.h>

void BaseModel::readMesh(const std::string& filename)
{
	/*if (!PMP::IO::read_polygon_mesh(filename, mesh) || is_empty(mesh) || !is_triangle_mesh(mesh))
	{
		std::cerr << "Invalid input." << std::endl;
		exit(EXIT_FAILURE);
	}*/
	igl::read_triangle_mesh(filename, V, F);
}

Eigen::Vector3d point2MeshProjection(const Eigen::Vector3d& point, const BaseModel& model)
{
	typedef nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXd> KDTree;
	KDTree tree(3, model.V, 10 /* leaf size */);

	const int max_range = 20;

	// do a knn search
	const size_t        num_results = 1;
	std::vector<size_t> ret_indexes(num_results);
	std::vector<int>  out_dists_sqr(num_results);

	nanoflann::KNNResultSet<int> resultSet(num_results);

	resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);
	tree.index_->findNeighbors(resultSet, point.data());

	std::ofstream out("./temp.xyz");
	out << model.V.row(ret_indexes[0]) << std::endl;

	for (size_t i = 0; i < resultSet.size(); i++)
		std::cout << "ret_index[" << i << "]=" << ret_indexes[i]
		<< " out_dist_sqr=" << out_dists_sqr[i] << std::endl;
}
