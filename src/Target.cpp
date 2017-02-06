#include <iostream>
#include <cmath>
#include <random>

#include <Eigen/Dense>

#include "Lie_group.h"
#include "World.h"
#include "Target.h"

// using namespace std;
// using namespace Eigen;


namespace Attitude_estimation{

	/* ########## Target (base class for all targets) definitions ########## */
	void Target::initialize(){
		state = Eigen::MatrixXd::Zero(world->get_state_dim(), world->get_time());
		angular_velocity = Eigen::MatrixXd::Zero(world->get_space_dim(), world->get_time());
		state.col(0) = IC; // Deterministic initialization is used by default
	}


	void Target::move(int TI, double dt, std::default_random_engine &generator){

		state.col(TI+1) = state.col(TI); // Target does not move for base class
	}


	/* ########## Sinusoidal target definitions ########## */
	void Sinusoidal::move(int TI, double dt, std::default_random_engine &generator){
		double t = TI*dt;
		angular_velocity(0,TI) = std::sin(2*M_PI*t/15.0);
		angular_velocity(1,TI) = -std::sin(2*M_PI*t/18.0 + M_PI/20.0);
		angular_velocity(2,TI) = std::cos(2*M_PI*t/17.0);

		// Add process noise in EACH velocity direction
		Eigen::VectorXd process_noise = Eigen::VectorXd::Constant(world->get_space_dim(),0);
		for (int d=0; d<angular_velocity.rows(); ++d){
			std::normal_distribution<double> distribution(0, process_noise_std(d)*sqrt(dt));
			process_noise(d) = distribution(generator);
		}

		state.col(TI+1) = dq(state.col(TI), angular_velocity.col(TI)*dt + process_noise); // Norm is 1
		
	}


}





