#pragma once
#include "BaseModel.h"

class ParticalMesh :public BaseModel
{
private:
	double c_theta = 0.3;
	double embedOmiga;
	double theta;

	size_t numParticles;
	Eigen::MatrixXd particleArray;

public:
	ParticalMesh(const double& _c_ctheta, const std::string& filename) : c_theta(_c_ctheta), BaseModel(filename)
	{
		embedOmiga = totalArea;
		theta = c_theta * std::sqrt(embedOmiga / m_V.rows());
		numParticles = m_V.rows();
		particleArray.resize(numParticles, 3);
	}

	void launchApp(const int& iterations, const int& numSearch);

private:
	void lbfgs_optimization(const int& iterations, const int& numSearch);
};