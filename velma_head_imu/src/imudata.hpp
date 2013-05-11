#ifndef IMUDATA_HPP
#define IMUDATA_HPP

#include <Eigen/Eigen>

struct ImuData
{
	Eigen::Vector3d angularVelocity;
	Eigen::Vector3d angularAcceleration;
	Eigen::Vector3d linearAcceleration;
};

#endif /* IMUDATA_HPP */

