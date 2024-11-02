#include "BaseModel.h"
//#include "FCPWHelper.h"
#include <vector>
#include <fstream>
#include <iostream>
//#include <CGAL/property_map.h>
//#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
//namespace PMP = CGAL::Polygon_mesh_processing;
#include <igl/knn.h>
#include <igl/read_triangle_mesh.h>
#include <igl/principal_curvature.h>
#include <igl/point_mesh_squared_distance.h>

BaseModel::BaseModel(const std::string& filename)
{
	igl::read_triangle_mesh(filename, m_V, m_F);
	//model2UnitCube();
	for (int i = 0; i < m_F.rows(); i++)
	{
		modelTris.emplace_back(Triangle<Eigen::Vector3d>(m_V.row((m_F.row(i))[0]),
			m_V.row((m_F.row(i))[1]),
			m_V.row((m_F.row(i))[2])
		));
		// �������������ε������
		totalArea += modelTris[i].area;
	}
	//initPQPModel();
	aabbTree.init(m_V, m_F);
	igl::per_vertex_normals(m_V, m_F, m_VN);
	igl::per_face_normals_stable(m_V, m_F, m_FN);
	igl::principal_curvature(m_V, m_F, PD1, PD2, PV1, PV2);
}

void BaseModel::readMesh(const std::string& filename)
{
	/*if (!PMP::IO::read_polygon_mesh(filename, mesh) || is_empty(mesh) || !is_triangle_mesh(mesh))
	{
		std::cerr << "Invalid input." << std::endl;
		exit(EXIT_FAILURE);
	}*/
	igl::read_triangle_mesh(filename, m_V, m_F);
}

// ����to unit cube
Eigen::Matrix4d BaseModel::calcUnitCubeTransformMatrix(const double& scaleFactor)
{
	Eigen::RowVector3d boxMin = m_V.colwise().minCoeff();
	Eigen::RowVector3d boxMax = m_V.colwise().maxCoeff();

	// Get the target solveRes (along the largest dimension)
	double scale = boxMax[0] - boxMin[0];
	double minScale = scale;
	for (int d = 1; d < 3; d++)
	{
		scale = std::max<double>(scale, boxMax[d] - boxMin[d]);
		minScale = std::min<double>(scale, boxMax[d] - boxMin[d]);
	}
	// std::cout << 1.1 + scale / minScale << std::endl;
	// scaleFactor =
	scale *= scaleFactor;
	Eigen::Vector3d center = 0.5 * boxMax + 0.5 * boxMin;

	for (int i = 0; i < 3; i++)
		center[i] -= scale / 2;
	Eigen::Matrix4d zoomMatrix = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d transMatrix = Eigen::Matrix4d::Identity();
	for (int i = 0; i < 3; i++)
	{
		zoomMatrix(i, i) = 1. / scale;
		transMatrix(3, i) = -center[i];
	}
	return zoomMatrix * transMatrix;
}

void BaseModel::model2UnitCube(const double& scaleFactor)
{
	auto transMat = calcUnitCubeTransformMatrix(scaleFactor);
	for (int i = 0; i < m_V.rows(); ++i)
	{
		m_V.row(i) += transMat.block(3, 0, 1, 3);
		m_V.row(i) = m_V.row(i) * transMat.block(0, 0, 3, 3);
	}
}

Eigen::MatrixXd BaseModel::getClosestPoint(const Eigen::MatrixXd& queryPointMat)
{
	Eigen::VectorXd sqrD;
	Eigen::VectorXi I;
	Eigen::MatrixXd C;

	// the output sqrD contains the (unsigned) squared distance from each point in P 
	// to its closest point given in C which lies on the element in F given by I
	aabbTree.squared_distance(m_V, m_F, queryPointMat, sqrD, I, C);

	return C;
}

void BaseModel::getClosestPoint(const Eigen::MatrixXd& queryPointMat,
	Eigen::VectorXd& sqrD, Eigen::VectorXi& I, Eigen::MatrixXd& C)
{
	// the output sqrD contains the (unsigned) squared distance from each point in P 
	// to its closest point given in C which lies on the element in F given by I
	aabbTree.squared_distance(m_V, m_F, queryPointMat, sqrD, I, C);
}

Eigen::MatrixXd BaseModel::getPointNormal(const Eigen::MatrixXd& queryPointMat)
{
	const size_t numPoint = queryPointMat.rows();

	Eigen::VectorXd sqrD;
	Eigen::VectorXi I;
	Eigen::MatrixXd C;
	getClosestPoint(queryPointMat, sqrD, I, C);

	Eigen::MatrixXd resNormal(numPoint, 3);
	for (int i = 0; i < numPoint; ++i)
	{
		Eigen::Vector3d normal = m_FN.row(I(i)).normalized();
		resNormal.row(i) = normal;
	}
	return resNormal;
}

Eigen::MatrixXd BaseModel::getPointNormal(const Eigen::MatrixXd& queryPointMat, Eigen::MatrixXd& C)
{
	const size_t numPoint = queryPointMat.rows();

	Eigen::VectorXd sqrD;
	Eigen::VectorXi I;
	getClosestPoint(queryPointMat, sqrD, I, C);

	Eigen::MatrixXd resNormal(numPoint, 3);
	for (int i = 0; i < numPoint; ++i)
	{
		Eigen::Vector3d normal = m_FN.row(I(i)).normalized();
		resNormal.row(i) = normal;
	}
	return resNormal;
}

Eigen::MatrixXd BaseModel::getPointPD(const Eigen::MatrixXd& queryPointMat)
{
	const size_t numPoint = queryPointMat.rows();

	Eigen::VectorXi indices;
	Eigen::VectorXd distances;

	Eigen::MatrixXd resPD(numPoint * 2, 3);
	return Eigen::MatrixXd();
}

//void BaseModel::initPQPModel()
//{
//	pqpModel = new PQP_Model();
//	pqpModel->BeginModel();
//
//	for (int f_id = 0; f_id < modelTris.size(); f_id++)
//	{
//		PQP_REAL tri[3][3];
//		tri[0][0] = modelTris[f_id].p1(0), tri[0][1] = modelTris[f_id].p1(1), tri[0][2] = modelTris[f_id].p1(2);
//		tri[1][0] = modelTris[f_id].p2(0), tri[1][1] = modelTris[f_id].p2(1), tri[1][2] = modelTris[f_id].p2(2);
//		tri[2][0] = modelTris[f_id].p3(0), tri[2][1] = modelTris[f_id].p3(1), tri[2][2] = modelTris[f_id].p3(2);
//		pqpModel->AddTri(tri[0], tri[1], tri[2], f_id);
//	}
//
//	pqpModel->EndModel();
//}
//
//Eigen::Vector3d pqp_distanceFromPointToModel(const Eigen::Vector3d& queryPoint, const BaseModel& model)
//{
//	PQP_DistanceResult res;
//	double point[3] = { queryPoint(0), queryPoint(1), queryPoint(2) };
//	PQP_Distance(&res, model.pqpModel, point, 0.0, 0.0);
//
//	/*std::ofstream out("./pqp_temp.xyz");
//	out << Eigen::Vector3d(res.p1[0], res.p2[1], res.p1[2]).transpose() << std::endl;*/
//
//	return Eigen::Vector3d(res.p1[0], res.p2[1], res.p1[2]); // res.p1��ͶӰ�㣬res.p2��queryPoint
//}
