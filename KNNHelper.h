#pragma once
#include <nanoflann.hpp>
#include <Eigen/Dense>

typedef nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXd> KDTree;

namespace knn_helper
{
	inline void getClosestPoint(const KDTree& kdTree, const std::vector<Eigen::Vector3d>& queryPointList,
		const Eigen::MatrixXd& modelVerts, std::vector<Eigen::Vector3d>& resPointList)
	{
		// do a knn search
		const size_t        num_results = queryPointList.size();
		std::vector<size_t> ret_indexes(num_results);
		std::vector<int>    out_dists_sqr(num_results);

		nanoflann::KNNResultSet<int> resultSet(num_results);

		for (size_t i = 0; i < num_results; ++i)
		{
			resultSet.init(&ret_indexes[i], &out_dists_sqr[i]);
			kdTree.index_->findNeighbors(resultSet, queryPointList[i].data());
			resPointList.emplace_back(modelVerts.row(ret_indexes[i]));
		}
	}

	inline void getNeighborPoint(const KDTree& kdTree, const Eigen::Vector3d& queryPoint,
		const Eigen::MatrixXd& m_V, const size_t num_results, std::vector<Eigen::Vector3d>& neighPointList)
	{
		// do a knn search
		std::vector<size_t> ret_indexes(num_results);
		std::vector<int>    out_dists_sqr(num_results);

		nanoflann::KNNResultSet<int> resultSet(num_results);

		for (size_t i = 0; i < num_results; ++i)
		{
			resultSet.init(&ret_indexes[i], &out_dists_sqr[i]);
			kdTree.index_->findNeighbors(resultSet, queryPoint.data());
			neighPointList.emplace_back(m_V.row(ret_indexes[i]));
		}
	}
}