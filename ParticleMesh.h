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
	ParticalMesh(const std::string& filename) : BaseModel(filename)
	{
		embedOmiga = totalArea;
		theta = c_theta * std::sqrt(embedOmiga / m_V.rows());
		numParticles = m_V.rows(); // for simplicity
		particleArray = m_V;
	}

	ParticalMesh(const std::string& filename, const double& _c_theta) : ParticalMesh(filename)
	{
		c_theta = _c_theta;
		theta = c_theta * std::sqrt(embedOmiga / m_V.rows());
	}

	void launchApp(const int& maxIterations, const int& numSearch, const std::string& out_file = "");

private:
	void lbfgs_optimization(const int& maxIterations, const int& numSearch, const std::string& out_file);
};