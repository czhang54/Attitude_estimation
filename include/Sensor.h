
/* This header defines all types of the sensor objects */

#ifndef SENSOR
#define SENSOR

#include <iostream>
#include <random>

#include <Eigen/Dense>

#include "Lie_group.h"

// using namespace std;
// using namespace Eigen;


namespace attitude_estimation{

	using namespace Eigen;

	// Forward declaration
	class World;
	class TargetBase;

	/* ########## Base class for all sensors ########## */
	class SensorBase
	{
	protected:

		World *world; // A pointer to the World, set when the sensor is initialized
		int dim; // Dimension of the measurement, e.g. dim=3 for gyroscope, accelerometer and magnetometer
		MatrixXd measurements; // Raw measurement data obtained at all times
		VectorXd noise_std; // Sensor noise parameter for each measurement component

	public:

		// Constructor
		SensorBase(int dim_sensor, const VectorXd &noise_std)
			: dim(dim_sensor), noise_std(noise_std) {};

		// Give access to acquire the measurement data, used by filters
		MatrixXd& get_measurements() {return measurements;}

		// Give access to acquire the noise parameters, used by filters
		VectorXd& get_sensor_noise() {return noise_std;}

		// Initialize the sensor
		void initialize(); 

		// A model that transform target state to measurement
		virtual VectorXd model(const quaternion &q);

		// Jacobian matrix of sensor model, used by Kalman filters but not particle filters
		virtual MatrixXd jacobian(const quaternion &q);

		// Generate measurements at ALL times
		virtual void observe(int TI, double dt, std::default_random_engine &generator);

		friend std::ostream& operator<<(std::ostream &out, const SensorBase *s){
			return s->message(out);
		}

		virtual std::ostream& message(std::ostream &out) const;

		friend World;

	};


	/* Accelerometer class */
	class Accelerometer: public SensorBase
	{
		Vector3d gravity; // Gravity vector

	public:

		// Constructor
		Accelerometer(int dim_sensor, const VectorXd &noise_std, const Vector3d &gravity)
			: SensorBase(dim_sensor, noise_std), gravity(gravity) {}

		// Model of accelerometer
		virtual VectorXd model(const quaternion &q) override;

		// Jacobian matrix of the model
		virtual MatrixXd jacobian(const quaternion &q) override;

		// Generate measurements at ALL times
		virtual void observe(int TI, double dt, std::default_random_engine &generator) override;

		virtual std::ostream& message(std::ostream &out) const override;

	};


	class Magnetometer: public SensorBase{

		Vector3d magnetic_field;

	public:

		// Constructor
		Magnetometer(int dim_sensor, const VectorXd &noise_std, const Vector3d &magnetic_field)
			: SensorBase(dim_sensor, noise_std), magnetic_field(magnetic_field) {}

		// Model of magnetometer
		virtual VectorXd model(const quaternion &q) override;

		// Jacobian matrix of the model
		virtual MatrixXd jacobian(const quaternion &q) override;

		// Generate measurements at ALL times
		virtual void observe(int TI, double dt, std::default_random_engine &generator) override;

		virtual std::ostream& message(std::ostream &out) const override;

	};

} // End of namespace attitude_estimation















#endif