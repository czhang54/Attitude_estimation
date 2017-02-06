#include <iostream>
#include <cmath>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include "Lie_group.h"

// using namespace std;
using namespace Eigen;


// Compute product of two quaternions
quaternion quaternion_product(const quaternion &q1, const quaternion &q2){
	quaternion q;
	Vector3d qv1 = q1.tail(3);
	Vector3d qv2 = q2.tail(3);
	q(0) = q1(0)*q2(0) - qv1.dot(qv2);
	q.tail(3) = q1(0)*qv2 + q2(0)*qv1 + qv1.cross(qv2);
	return q;
}

// Update quaternion according to angular velocity vector defined in body/global frame
quaternion dq(const quaternion &q, const Vector3d &omega, std::string frame){
	quaternion exp;
	double norm = omega.norm();
	exp(0) = std::cos(norm/2.0);
	exp.tail(3) = omega*sin(norm/2.0)/norm;
	if (frame.compare("body_frame") == 0){
		return quaternion_product(q, exp);
	}
	else{
		return quaternion_product(exp, q);
	}
	
}

// Convert quaternion to rotation matrix
Matrix3d q2R(const quaternion &q){
	Matrix3d R = Matrix3d::Zero();
	for (int d=0; d<3; ++d){
		R(d,d) = q(0)*q(0) + q(d)*q(d) - 0.5;
	}
	R(0,1) = q(1)*q(2) - q(0)*q(3);
	R(1,0) = q(1)*q(2) + q(0)*q(3);
	R(0,2) = q(1)*q(3) + q(0)*q(2);
	R(2,0) = q(1)*q(3) - q(0)*q(2);
	R(1,2) = q(2)*q(3) - q(0)*q(1);
	R(2,1) = q(2)*q(3) + q(0)*q(1);

	return 2*R;
}

// Compute skew-symmetric matrix from given vector
Matrix3d skew(const Vector3d &v){
	Matrix3d M = Matrix3d::Zero();
	M(2,1) = v(0);
	M(1,2) = -v(0);
	M(0,2) = v(1);
	M(2,0) = -v(1);
	M(1,0) = v(2);
	M(0,1) = -v(2);

	return M;
}

// Create identity matrix of given size
MatrixXd identity(int d){
	return MatrixXd::Identity(d,d);
}

// Convert MRP to quaternion
quaternion MRP2q(const Vector3d &m){
	quaternion q = quaternion::Zero();
	double norm = m.squaredNorm();
	q(0) = (16-norm)/(16+norm);
	q.tail(3) = m*8/(16+norm);

	return q;
}

// Compute inverse quaternion
quaternion inverse_quaternion(const quaternion &q){
	quaternion q_inv = quaternion::Zero();
	q_inv(0) = q(0);
	q_inv.tail(3) = -q.tail(3);

	return q_inv;
}

// Compute quaternion from angle-axis parameter
quaternion angle_axis(const Vector3d axis, double angle){
	quaternion q = quaternion::Zero();
	double theta = angle*M_PI/180.0; 
	Vector3d omega = axis/(axis.norm());
	q(0) = std::cos(theta/2.0);
	q.tail(3) = omega*std::sin(theta/2.0);

	return q;
}

// Compute statistical mean of quaternion samples
quaternion quaternion_mean(MatrixXd &quaternions){
	MatrixXd M = quaternions*(quaternions.transpose())/(static_cast<double>(quaternions.cols()));
	// Solve eigenvalues and eigenvectors
	SelfAdjointEigenSolver<MatrixXd> solver(M);
	ArrayXd eigenvalues = solver.eigenvalues().array();
	ArrayXd::Index max_loc;
	double max_eigenvalue = eigenvalues.maxCoeff(&max_loc);

	return solver.eigenvectors().col(max_loc);
}


