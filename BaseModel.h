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
//#include "FCPWHelper.h"
#include <PQP/Build.h>
#include <igl/AABB.h>

template <typename Real>
struct Triangle
{
	Real p1, p2, p3;
	Real normal;
	double area;

	Triangle() {}

	Triangle(const Real& _p1, const Real& _p2, const Real& _p3) :p1(_p1), p2(_p2), p3(_p3) { area = computeTriangleArea(); }

	double computeTriangleArea()
	{
		Real v12 = p2 - p1;
		Real v13 = p3 - p1;
		return 0.5 * (v12.cross(v13)).norm();
	}
};

//#pragma comment(lib,"PQP")

struct BaseModel
{
public:
	//Mesh mesh;
	Eigen::MatrixXd m_V;
	Eigen::MatrixXd m_VN;
	Eigen::MatrixXi m_F;
	Eigen::MatrixXd m_FN;

	std::vector<Triangle<Eigen::Vector3d>> modelTris;

	igl::AABB<Eigen::MatrixXd, 3> aabbTree;

	double totalArea = 0.0;

	Eigen::MatrixXd PD1, PD2; // PD1�洢��������ʷ���PD2�洢��С�����ʷ���
	Eigen::VectorXd PV1, PV2; // PV1�洢��������ʣ�PV2�洢��С������

	//PQP_Model* pqpModel;

	//fcpw::Scene<3> scene;

public:
	BaseModel(const std::string& filename);

	~BaseModel() {}

	void readMesh(const std::string& filename);

private:
	Eigen::Matrix4d calcUnitCubeTransformMatrix(const double& scaleFactor);

public:
	void model2UnitCube(const double& scaleFactor = 1.0);

	Eigen::MatrixXd getClosestPoint(const Eigen::MatrixXd& queryPointMat);

	void getClosestPoint(const Eigen::MatrixXd& queryPointMat,
		Eigen::VectorXd& sqrD, Eigen::VectorXi& I, Eigen::MatrixXd& C);

	Eigen::MatrixXd getPointNormal(const Eigen::MatrixXd& queryPointMat);

	Eigen::MatrixXd getPointNormal(const Eigen::MatrixXd& queryPointMat, Eigen::MatrixXd& C);

	Eigen::MatrixXd getPointPD(const Eigen::MatrixXd& queryPointMat);

	//void initPQPModel();
};

//Eigen::Vector3d pqp_distanceFromPointToModel(const Eigen::Vector3d& pqpModel, const BaseModel& model);