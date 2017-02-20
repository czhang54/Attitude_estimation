
/* Implementation of: 
   (1) base class of all filters, 
   (2) base class of all Kalman filters, and 
   (3) base class of all particle filters. */

#include <iostream>
#include <cmath>
#include <random>

#include <Eigen/Dense>

#include "World.h"
#include "Target.h"
#include "Sensor.h"
#include "Filter.h"
#include "Lie_group.h"

// using namespace std;
// using namespace Eigen;


namespace attitude_estimation{

	using namespace Eigen;

	/* ########## FilterBase (base class for all filters) implementations ########## */

	void FilterBase::initialize(std::default_random_engine &generator) {
		dim_filter = IC_mean.size();
		dim_space = IC_std.size();
		estimates = MatrixXd::Zero(dim_filter, world->get_time());
		angle_error = VectorXd::Zero(world->get_time());
		estimates.col(0) = IC_mean;

		// Compute total dimension of all sensor measurements
		for (int i=0; i<world->get_sensor().size(); ++i){
			dim_sensor += world->get_sensor()[i]->get_sensor_noise().size();
		}
	}

	void FilterBase::update(int TI, double dt, std::default_random_engine &generator){
		estimates.col(TI+1) = estimates.col(TI);
	}

	void FilterBase::performance(int TI){
		// Evaluate estimation error according to some performance measure
		quaternion target_state = world->get_target()->get_state().col(TI);
		quaternion estimate_error = quaternion_product(inverse_quaternion(estimates.col(TI)), target_state);
		angle_error(TI) = 2*std::acos(std::abs(estimate_error(0)))*180/3.14;
		std::cout << "angle error = " << angle_error(TI) << '\n';
	}

	std::ostream& operator<<(std::ostream &out, const FilterBase *f){
		return f->message(out);
	}

	std::ostream& FilterBase::message(std::ostream &out) const {
		out << "A filter will be implemented" << '\n';
		return out;
	}


	/* ########## KalmanFilterBase (base class for all Kalman-type filters) implementations ########## */
	void KalmanFilterBase::initialize(std::default_random_engine &generator){

		FilterBase::initialize(generator); 

		VectorXd IC_cov = IC_std.array().square(); // square().asDiagonal() is not allowed!
		covariance = IC_cov.asDiagonal(); // Initialize filter covariance, updated using MEKF::update

		// Compute noise matrices Q_d, Q_v
		VectorXd process_noise_cov = world->get_target()->get_process_noise().array().square();
		Q_d = process_noise_cov.asDiagonal();

		int dim_sum = 0;
		Q_v = MatrixXd::Zero(dim_sensor, dim_sensor);
		for (int i=0; i<world->get_sensor().size(); ++i){
			VectorXd sensor_noise_cov = world->get_sensor()[i]->get_sensor_noise().array().square();
			int dim = sensor_noise_cov.size();
			Q_v.block(dim_sum, dim_sum, dim, dim) = sensor_noise_cov.asDiagonal();
			dim_sum += dim;
		}
	}



	/* ########## ParticleFilterBase (base class for all particle-based filters) implementations ########## */

	void ParticleFilterBase::initialize(std::default_random_engine &generator){

		FilterBase::initialize(generator);

		// Generate initial particles
		particles = MatrixXd::Zero(dim_filter, N);

		initialize_gaussian(generator); 
		// std::cout << "Initial particles generated" << '\n';

		sensor_noise_cov = VectorXd::Zero(dim_sensor);
		int count = 0;
		for (SensorBase *s: world->get_sensor()){
			int dim = s->get_sensor_noise().size();
			sensor_noise_cov.segment(count, dim) = s->get_sensor_noise().array().square();
			count += dim;
		}
	}

	void ParticleFilterBase::initialize_gaussian(std::default_random_engine &generator){
		// First generate samples in Lie algebra (LA), and then project onto Lie group using exponential map

		for (int i=0; i<N; i++){	
			VectorXd particle_LA(dim_space);
			for (int d=0; d<dim_space; d++){
				std::normal_distribution<double> normal(0, IC_std(d));
				particle_LA(d) = normal(generator);
			}
			particles.col(i) = dq(IC_mean, particle_LA);
		}
	}

	std::ostream& ParticleFilterBase::message(std::ostream &out) const {
			out << "Particle filters will be implemented" << '\n';
			return out;
		}


} // End of namespace attitude_estimation
