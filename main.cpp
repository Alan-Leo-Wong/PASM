#include <fstream>
#include <iostream>
#include "BaseModel.h"
#include "KNNHelper.h"

void outFile(const Eigen::MatrixXd& queryPointMat, const Eigen::MatrixXd& resPointMat)
{
	std::ofstream out("./query.xyz");
	for (size_t i = 0; i < queryPointMat.rows(); ++i)
		out << queryPointMat.row(i) << std::endl;
	out.close();
	out.open("./igl_res.xyz");
	for (size_t i = 0; i < resPointMat.rows(); ++i)
		out << resPointMat.row(i) << std::endl;
	out.close();
}

void testClosestPoint()
{
	BaseModel model("./model/bunny.off");

	Eigen::MatrixXd queryPointMat(3, 3);
	Eigen::VectorXd sqrD;
	Eigen::VectorXi I;
	Eigen::MatrixXd C;
	queryPointMat << 0, 0, 0,
		0.1, 0, 0,
		0, 0.1, 0;
	model.getClosestPoint(queryPointMat, sqrD, I, C);
	outFile(queryPointMat, C);
}

void testPointNormal()
{
	BaseModel model("./model/bunny.off");

	Eigen::MatrixXd queryPointMat(3, 3);
	queryPointMat << 0, 0, 0,
		0.1, 0, 0,
		0, 0.1, 0;
	auto normal = model.getPointNormal(queryPointMat);
	std::cout << normal << std::endl;
}

int main()
{
	BaseModel model("./model/bunny.off");

	//testPointNormal();

	//KDTree aabbTree(3, model.V, 10);

	//knn_helper::getClosestPoint(tree, queryPointList, model.V, resPointList);

	return 0;
}