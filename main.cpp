#include <iostream>
#include "BaseModel.h"

int main()
{
	BaseModel model("./model/bunny.off");

	point2MeshProjection(Eigen::Vector3d(0, 0, 0), model);

	return 0;
}