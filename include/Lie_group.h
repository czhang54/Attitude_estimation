#ifndef LIE_GROUP
#define LIE_GROUP

#include <iostream>
#include <cmath>
#include <string>

#include <Eigen/Dense>

// using namespace std;
// using namespace Eigen;

/* This header file declares functions related to computations using quaternions and/or rotation matrices */

typedef Eigen::Vector4d quaternion; // One can alternatively use the Quaternion class provided by Eigen/Geometry
typedef Eigen::Matrix3d RotationMatrix;

// Compute product of two quaternions
quaternion quaternion_product(const quaternion &q1, const quaternion &q2);

// Update quaternion according to angular velocity vector defined in body/global frame
quaternion dq(const quaternion &q, const Eigen::Vector3d &omega, std::string frame="body_frame");

// Convert quaternion to rotation matrix
RotationMatrix q2R(const quaternion &q);

// Compute skew-symmetric matrix from given vector
Eigen::Matrix3d skew(const Eigen::Vector3d &v);

// Create identity matrix of given size
Eigen::MatrixXd identity(int d);

// Convert MRP to quaternion
quaternion MRP2q(const Eigen::Vector3d &m);

// Compute inverse quaternion
quaternion inverse_quaternion(const quaternion &q);

// Compute quaternion from angle-axis parameter
quaternion angle_axis(const Eigen::Vector3d axis, double angle);

// Compute statistical mean of quaternion samples
quaternion quaternion_mean(Eigen::MatrixXd &quaternions);


#endif
