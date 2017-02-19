#include <iostream>
#include <random>

#include <Eigen/Dense>

#include "Lie_group.h" 
#include "World.h"
#include "Target.h"
#include "Sensor.h"

// using namespace std;
// using namespace Eigen;


namespace attitude_estimation{

	using namespace Eigen;

	/* ########## Base class for all sensors ########## */
	void SensorBase::initialize(){
		measurements = MatrixXd::Zero(dim, world->get_time());
	}

	VectorXd SensorBase::model(const quaternion &q){
		Vector3d Y = Vector3d::Zero();
		return Y;
	}

	MatrixXd SensorBase::jacobian(const quaternion &q){
		MatrixXd H = MatrixXd::Zero(dim, 3);
		return H;
	}

	void SensorBase::observe(int TI, double dt, std::default_random_engine &generator){
		measurements.col(TI) = VectorXd::Zero(dim); // No measurement is made for base class
	}

	std::ostream& SensorBase::message(std::ostream &out) const {
		out << "Sensor is used" << '\n';
		return out;
	}


	/* ########## Accelerometer definitions ########## */
	// Y_t = -R^T*g + W_t
	void Accelerometer::observe(int TI, double dt, std::default_random_engine &generator) {

		quaternion target_state = world->get_target()->get_state().col(TI);

		// Add sensor noise
		VectorXd sensor_noise = VectorXd::Zero(dim);
		for (int d=0; d<dim; ++d){
			std::normal_distribution<double> distribution(0, noise_std(d));
			sensor_noise(d) = distribution(generator);
		}

		measurements.col(TI) = model(target_state) + sensor_noise;
	}

	VectorXd Accelerometer::model(const quaternion &q){

		return -(q2R(q).transpose())*gravity;
	}

	MatrixXd Accelerometer::jacobian(const quaternion &q){
		Matrix3d H = skew(model(q));
		return H;
	}

	std::ostream& Accelerometer::message(std::ostream &out) const {
		out << "Accelerometer is used" << '\n';
		return out;
	}


	/* ########## Magnetometer definitions ########## */
	// Y_t = R^T*b + W_t
	void Magnetometer::observe(int TI, double dt, std::default_random_engine &generator) {
		
		quaternion target_state = world->get_target()->get_state().col(TI);

		// Add sensor noise
		VectorXd sensor_noise = VectorXd::Zero(dim);
		for (int d=0; d<dim; ++d){
			std::normal_distribution<double> distribution(0, noise_std(d));
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

	std::ostream& Magnetometer::message(std::ostream &out) const {
		out << "Magnetometer is used" << '\n';
		return out;
	}
 
} // End of namespace attitude_estimation




