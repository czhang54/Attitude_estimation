#ifndef SENSOR
#define SENSOR

#include <iostream>
#include <cmath>
#include <random>

#include <Eigen/Dense>

#include "Lie_group.h"

using namespace std;
using namespace Eigen;

class World;
class Target;

class Sensor
{
protected:

	World *world;
	int dim;
	MatrixXd measurements;
	VectorXd noise_std;

public:

	Sensor(int dim_sensor, const VectorXd &noise_std)
		: dim(dim_sensor), noise_std(noise_std) {};

	MatrixXd& get_measurements() {return measurements;}

	VectorXd& get_sensor_noise() {return noise_std;}

	void initialize(); // Set time horizon

	virtual VectorXd model(const quaternion &q);

	virtual MatrixXd jacobian(const quaternion &q);

	virtual void observe(int TI, double dt, default_random_engine &generator);

	friend ostream& operator<<(ostream &out, const Sensor *s){
		return s->message(out);
	}

	virtual ostream& message(ostream &out) const;

	friend World;

};


class Accelerometer: public Sensor
{
	Vector3d gravity;

public:

	Accelerometer(int dim_sensor, const VectorXd &noise_std, const Vector3d &gravity)
		: Sensor(dim_sensor, noise_std), gravity(gravity) {}

	virtual VectorXd model(const quaternion &q) override;

	virtual MatrixXd jacobian(const quaternion &q) override;

	virtual void observe(int TI, double dt, default_random_engine &generator) override;

	virtual ostream& message(ostream &out) const override;

};


class Magnetometer: public Sensor{

	Vector3d magnetic_field;

public:

	Magnetometer(int dim_sensor, const VectorXd &noise_std, const Vector3d &magnetic_field)
		: Sensor(dim_sensor, noise_std), magnetic_field(magnetic_field) {}

	virtual VectorXd model(const quaternion &q) override;

	virtual MatrixXd jacobian(const quaternion &q) override;

	virtual void observe(int TI, double dt, default_random_engine &generator) override;

	virtual ostream& message(ostream &out) const override;

};

















#endif