#include <iostream>
#include <cmath>
#include <random>

#include <Eigen/Dense>

#include "../include/Lie_group.h" // Need to include this, though it is also included in Target.cpp
#include "../include/World.h"
#include "../include/Target.h"
#include "../include/Sensor.h"

using namespace std;
using namespace Eigen;


void Sensor::initialize(){
	measurements = MatrixXd::Zero(dim, world->get_time());
}

VectorXd Sensor::model(const quaternion &q){
	Vector3d Y = Vector3d::Zero();
	return Y;
}

MatrixXd Sensor::jacobian(const quaternion &q){
	MatrixXd H = MatrixXd::Zero(dim, 3);
	return H;
}

void Sensor::observe(int TI, double dt, default_random_engine &generator){
	measurements.col(TI) = VectorXd::Zero(dim); // No measurement is made for base class
}

ostream& Sensor::message(ostream &out) const {
	out << "Sensor is used" << '\n';
	return out;
}

// Y_t = -R^T*g + W_t
void Accelerometer::observe(int TI, double dt, default_random_engine &generator) {
	// cout << "TI = " << TI << '\n';
	quaternion target_state = world->get_target()->get_state().col(TI);

	// Add sensor noise
	VectorXd sensor_noise = VectorXd::Zero(dim);
	// default_random_engine generator;
	// generator.seed(static_cast<unsigned int>(time(0)));
	// srand(unsigned int) time((0));
	for (int d=0; d<dim; ++d){
		normal_distribution<double> distribution(0, noise_std(d));
		sensor_noise(d) = distribution(generator);
	}

	measurements.col(TI) = model(target_state) + sensor_noise;
}

VectorXd Accelerometer::model(const quaternion &q){
	// Matrix3d R = q2R(q);
	return -(q2R(q).transpose())*gravity;
}

MatrixXd Accelerometer::jacobian(const quaternion &q){
	Matrix3d H = skew(model(q));
	return H;
}

ostream& Accelerometer::message(ostream &out) const {
	out << "Accelerometer is used" << '\n';
	return out;
}


void Magnetometer::observe(int TI, double dt, default_random_engine &generator) {
	
	quaternion target_state = world->get_target()->get_state().col(TI);

	// Add sensor noise
	VectorXd sensor_noise = VectorXd::Zero(dim);
	for (int d=0; d<dim; ++d){
		normal_distribution<double> distribution(0, noise_std(d));
		sensor_noise(d) = distribution(generator);
	}

	measurements.col(TI) = model(target_state) + sensor_noise;
}

VectorXd Magnetometer::model(const quaternion &q){
	return (q2R(q).transpose())*magnetic_field;
}

MatrixXd Magnetometer::jacobian(const quaternion &q){
	Matrix3d H = skew(model(q));
	return H;
}

ostream& Magnetometer::message(ostream &out) const {
	out << "Magnetometer is used" << '\n';
	return out;
}






