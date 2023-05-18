#pragma once
#define FCPW_USE_ENOKI
#define FCPW_SIMD_WIDTH 4
#include <fcpw/fcpw.h>
#include <vector>
#include <Eigen/Dense>

namespace fcpw_helper
{
	inline Eigen::Vector3d getClosestPoint(const Eigen::Vector3d& queryPoint, const fcpw::Scene<3>& scene)
	{
		fcpw::Interaction<3> interaction;
		scene.findClosestPoint(queryPoint, interaction);
		return interaction.pqpModel;
	}

	inline double getSignedDistance(const Eigen::Vector3d& queryPoint, const fcpw::Scene<3>& scene)
	{
		// perform a closest point query
		fcpw::Interaction<3> interaction;
		scene.findClosestPoint(queryPoint, interaction);
		return interaction.signedDistance(queryPoint);
	}

	inline void initSDF(fcpw::Scene<3>& scene, const Eigen::MatrixXd& modelVerts, const Eigen::MatrixXi& modelFaces)
	{
		// set the types of primitives the objects in the scene contain;
		// in this case, we have a single object consisting of only triangles
		scene.setObjectTypes({ {fcpw::PrimitiveType::Triangle} });

		// set the vertex and triangle count of the (0th) object
		size_t nVertices = modelVerts.size();
		size_t nTriangles = modelFaces.size();
		scene.setObjectVertexCount(nVertices, 0);
		scene.setObjectTriangleCount(nTriangles, 0);

		// specify the vertex positions
		for (size_t i = 0; i < nVertices; i++)
			scene.setObjectVertex(modelVerts.row(i), i, 0);

		// specify the triangle indices
		for (size_t i = 0; i < nTriangles; i++)
			scene.setObjectTriangle(modelFaces.row(i).data(), i, 0);

		scene.computeObjectNormals(0);

		// now that the geometry has been specified, build the acceleration structure
		scene.build(fcpw::AggregateType::Bvh_SurfaceArea, true); // the second boolean argument enables vectorization
	}
}