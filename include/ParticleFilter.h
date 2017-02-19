
/* Define the traditional particle filter (PF) algorithm */

#ifndef PARTICLEFILTER
#define PARTICLEFILTER

#include <iostream>

#include <Eigen/Dense>

#include "Filter.h"

namespace attitude_estimation{

	class ParticleFilter: public ParticleFilterBase
	{

		double diffuse_kernel; // Noise parameter to diffuse particles after resampling

	public:

		// Constructor
		ParticleFilter(const VectorXd &IC_mean, const VectorXd &IC_std, const int num_particles, const double diffuse_kernel)
			: ParticleFilterBase(IC_mean, IC_std, num_particles), diffuse_kernel(diffuse_kernel) {}

		// Run particle filter algorithm for one step
		virtual void update(int TI, double dt, std::default_random_engine &generator) override;

		// Resampling procedure
		void resampling(VectorXd &weights, std::default_random_engine &generator);
	 
		virtual std::ostream& message(std::ostream &out) const override;

	};


} // End of attitude_estimation namespace


#endif