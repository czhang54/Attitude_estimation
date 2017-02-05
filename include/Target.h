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

class World; // Need to declare World class

/* ########## Target (base class for all targets) ########## */
class Target
{
protected:

	World *world; // A pointer to the World, set when the target is initialized
	MatrixXd state; // (Quaternion) state at all times
	MatrixXd angular_velocity; // Angular velocity vector at all times
	VectorXd IC; // Initial attitude of target
	VectorXd IC_std; // Standard deviation of initial distribution, not used if target initialization is deterministic
	VectorXd process_noise_std; // Noise parameter of process noise added to the target motion

public:

	Target(const VectorXd &IC, const VectorXd &IC_std, const VectorXd &process_noise_std)
		: IC(IC), IC_std(IC_std), process_noise_std(process_noise_std) {}
 
 	// Give access to the following information of target, used by sensors and filters
	MatrixXd& get_state() {return state;}
	MatrixXd& get_velocity() {return angular_velocity;}
	VectorXd& get_process_noise() {return process_noise_std;}

	// Initialize a target
	void initialize(); // Set time horizon and initial condition

	// Simulate a target according to a model
	virtual void move(int TI, double dt, default_random_engine &generator);

	friend ostream& operator<<(ostream &out, const Target *t){
		return t->message(out);
	}

	virtual ostream& message(ostream &out) const {
		out << "Tracking a target" << '\n';
		return out;
	}

	friend World; // MUST friend the entire World class!
	// friend World::add_target(Target *t);

};


/* A target whose angular velocity model is sinusoidal */
class Sinusoidal: public Target
{

public:

	Sinusoidal(const VectorXd &IC, const VectorXd &IC_std, const VectorXd &process_noise_std)
		: Target(IC, IC_std, process_noise_std) {}

	virtual void move(int TI, double dt, default_random_engine &generator) override;

	virtual ostream& message(ostream &out) const override {
		out << "Tracking a target with sinusoidal angular velocity..." << '\n';
		return out;
	}

};





















#endif