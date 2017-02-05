#ifndef LIE_GROUP
#define LIE_GROUP

#include <iostream>
#include <cmath>
#include <string>

#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

typedef Eigen::Vector4d quaternion;
typedef Eigen::Matrix3d RotationMatrix;

// Compute product of two quaternions
quaternion quaternion_product(const quaternion &q1, const quaternion &q2);

// Update quaternion according to angular velocity vector defined in body/global frame
quaternion dq(const quaternion &q, const Vector3d &omega, string frame="body_frame");

// Convert quaternion to rotation matrix
RotationMatrix q2R(const quaternion &q);

// Compute skew-symmetric matrix from given vector
Matrix3d skew(const Vector3d &v);

// Create identity matrix of given size
MatrixXd identity(int d);

// Convert MRP to quaternion
quaternion MRP2q(const Vector3d &m);

// Compute inverse quaternion
quaternion inverse_quaternion(const quaternion &q);

// Compute quaternion from angle-axis parameter
quaternion angle_axis(const Vector3d axis, double angle);

// Compute statistical mean of quaternion samples
quaternion quaternion_mean(MatrixXd &quaternions);


#endif