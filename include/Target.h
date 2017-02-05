#ifndef TARGET
#define TARGET

#include <iostream>
#include <string>
#include <vector>
#include <array>
#include <cmath>

#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

// typedef Vector4d quaternion;

class World; // Need to declare World class

class Target
{
protected:

	World *world; // Need declaration of World class as above
	MatrixXd state; // (4, num_times)
	MatrixXd angular_velocity; // (3, num_times)
	VectorXd IC;
	VectorXd IC_std;
	VectorXd process_noise_std;

public:

	Target(const VectorXd &IC, const VectorXd &IC_std, const VectorXd &process_noise_std)
		: IC(IC), IC_std(IC_std), process_noise_std(process_noise_std) {}
 
	MatrixXd& get_state() {return state;}

	MatrixXd& get_velocity() {return angular_velocity;}

	VectorXd& get_process_noise() {return process_noise_std;}

	void initialize(); // Set time horizon and initial condition

	virtual void move(int TI, double dt, default_random_engine &generator);

	// virtual VectorXd update(const VectorXd &q, int TI, double dt);

	friend ostream& operator<<(ostream &out, const Target *t){
		return t->message(out);
	}

	virtual ostream& message(ostream &out) const {
		out << "Consider a target" << '\n';
		return out;
	}

	friend World; // MUST friend the entire World class!
	// friend World::add_target(Target *t);

};


class Sinusoidal: public Target
{

public:

	Sinusoidal(const VectorXd &IC, const VectorXd &IC_std, const VectorXd &process_noise_std)
		: Target(IC, IC_std, process_noise_std) {}

	virtual void move(int TI, double dt, default_random_engine &generator) override;

	virtual ostream& message(ostream &out) const override {
		out << "Consider a target with sinusoidal angular velocity..." << '\n';
		return out;
	}

};





















#endif