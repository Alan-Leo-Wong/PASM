#pragma once
#include "BaseModel.h"
#include "KNNHelper.h"

struct APOM
{
	Eigen::MatrixXd before_particleMat;

	APOM(const Eigen::MatrixXd& _before_particleMat) :before_particleMat(_before_particleMat) {}

	double operator()(const Eigen::VectorXd& before_particle, Eigen::Vector3d& i_force)
	{
		KDTree kdTree(3, before_particleMat, 10);
		Eigen::MatrixXd normal = BaseModel::getPointNormal(before_particleMat);
	}
};