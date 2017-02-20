
/* Implementation of the traditional particle filter (PF) algorithm */

#include <iostream>
#include <cmath>
#include <random>

#include <Eigen/Dense>

#include "World.h"
#include "Target.h"
#include "Sensor.h"
#include "Filter.h"
#include "ParticleFilter.h"
#include "Lie_group.h"


namespace attitude_estimation{

	// Run particle filter algorithm for one step
	void ParticleFilter::update(int TI, double dt, std::default_random_engine &generator){

		std::cout << "Particle filter ";

		if (TI == 0){performance(TI);}

		/* PREDICTION */
		VectorXd target_velocity = world->get_target()->get_velocity().col(TI);
		VectorXd process_noise_std = world->get_target()->get_process_noise();
		// Add process noise
		VectorXd process_noise = VectorXd::Constant(dim_space, 0);
		for (int d=0; d<dim_space; ++d){
			std::normal_distribution<double> distribution(0, process_noise_std(d)*sqrt(dt));
			process_noise(d) = distribution(generator);
		}
		for (int i=0; i<N; ++i){
			// Prediction for each particle
			particles.col(i) = dq(particles.col(i), target_velocity*dt + process_noise);
		}

		/* MEASUREMENT UPDATE */
		// Compute importance weights
		VectorXd w = VectorXd::Zero(N);
		for (SensorBase *s: world->get_sensor()){
			VectorXd measurement = s->get_measurements().col(TI+1);
			VectorXd sensor_noise_std = s->get_sensor_noise();
			for (int i=0; i<N; ++i){
				VectorXd innovation = measurement - s->model(particles.col(i));
				VectorXd I = innovation.array()/sensor_noise_std.array();
				w(i) -= 0.5*I.squaredNorm();
			}
		}
		VectorXd weights = w.array().exp();
		double sum = weights.sum();
		weights /= sum; // Normalize weights

		// Resampling
		resampling(weights, generator);

		// Diffuse particles to maintain particle diversity
		for (int i=0; i<N; i++){	
			VectorXd particle_LA(dim_space);
			for (int d=0; d<dim_space; d++){
				std::normal_distribution<double> normal(0, diffuse_kernel_);
				particle_LA(d) = normal(generator);
			}
			particles.col(i) = dq(particles.col(i), particle_LA);
		}

		estimates.col(TI+1) = quaternion_mean(particles);

		performance(TI+1);

	}


	// Resampling procedure
	void ParticleFilter::resampling(VectorXd &weights, std::default_random_engine &generator){
		// Resampling with replacement
		MatrixXd resampled_particles = MatrixXd::Zero(dim_filter, N);

		std::uniform_real_distribution<double> uniform(0, 1/(static_cast<double>(N)));
		double r = uniform(generator);

		double c = weights(0);
		int count = 0;

		for (int n=0; n<N; ++n){
			double U = r + n/(static_cast<double>(N));
			while (U > c){
				count += 1;
				c += weights(count);
			}
			resampled_particles.col(n) = particles.col(count);
		}

		for (int i=0; i<N; ++i){
			particles.col(i) = resampled_particles.col(i);
		}

	}


	std::ostream& ParticleFilter::message(std::ostream &out) const {
		out << "Particle filter will be implemented" << '\n';
		return out;
	}


} // End of attitude_estimation namespace

