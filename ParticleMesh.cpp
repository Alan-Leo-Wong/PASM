#include "KNNHelper.h"
#include "ParticleMesh.h"
#include <LBFGS.h>

using namespace LBFGSpp;

void ParticalMesh::launchApp(const int& iterations, const int& numSearch)
{
	lbfgs_optimization(iterations, numSearch);
}

void ParticalMesh::lbfgs_optimization(const int& iterations, const int& numSearch)
{
	// solver's param
	LBFGSParam<Eigen::Vector3d> param;
	param.epsilon = Eigen::Vector3d(1e-6, 1e-6, 1e-6);
	param.max_iterations = iterations;

	auto optimize_fun = [=](const Eigen::Matrix<Eigen::Vector3d, -1, 1>& before_particle, Eigen::Vector3d& i_force)
	{
		Eigen::MatrixXd iter_particle(numParticles, 3);
		double fx = .0;

		const Eigen::MatrixXd _before_particle = before_particle;

		KDTree kdTree(3, _before_particle, 10);
		Eigen::MatrixXd normal = getPointNormal(before_particle);
		for (size_t i = 0; i < numParticles; ++i)
		{
			//Eigen::Vector3d i_force;

			const Eigen::Vector3d particleCoord = before_particle.row(i);
			std::vector<Eigen::Vector3d> neighPointList;
			knn_helper::getNeighborPoint(kdTree, particleCoord, before_particle, numSearch, neighPointList);
			for (size_t j = 0; j < neighPointList.size(); ++j)
			{
				const double& ij_energy = std::exp(-((particleCoord - neighPointList[j]).squaredNorm()) / (4 * theta * theta));
				fx += ij_energy;
				i_force += (particleCoord - neighPointList[j]) / (2 * theta * theta) * ij_energy;
			}

			// project to the surface
			i_force -= (i_force.dot(normal.row(i))) * normal.row(i);

			iter_particle.row(i) = before_particle.row(i) + i_force;
		}

		return fx;
	};

	// Create solver and function object
	LBFGSSolver<Eigen::Vector3d, LineSearchBracketing> solver(param);
	Eigen::Vector3d energy;
	Eigen::Matrix<Eigen::Vector3d, -1, 1> _particleArray = particleArray;
	solver.minimize(optimize_fun, _particleArray, energy);
}
