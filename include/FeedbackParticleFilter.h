
/* Define the feedback particle filter (FPF) algorithm. 
   FPF is recently developed for attitude estimation by my research group. */

#ifndef FEEDBACKPARTICLEFILTER
#define FEEDBACKPARTICLEFILTER

#include <iostream>

#include <Eigen/Dense>

#include "Filter.h"

namespace attitude_estimation{

	typedef Matrix<MatrixXd, Dynamic, 1> Tensor3Xd; // 3d array (tensor)

	/* Feedback particle filter */
	class FeedbackParticleFilter: public ParticleFilterBase
	{
		std::string gain_solver_; // Name of gain solver
		int TI_subdivide_; // Maximum number of filter steps that further sub-divided
		int num_subdivide_; // Number of subdivided steps within each normal step
	 
	public:

		// Constructor
		FeedbackParticleFilter(const VectorXd &IC_mean, const VectorXd &IC_std, const int num_particles, std::string gain_solver, int TI_subdivide=0, int num_subdivide=1)
			: ParticleFilterBase(IC_mean, IC_std, num_particles), gain_solver_(gain_solver), TI_subdivide_(TI_subdivide), num_subdivide_(num_subdivide) {}

		// Run FPF algorithm for one iteration
		virtual void update(int TI, double dt, std::default_random_engine &generator) override;

		// Galerkin method to solve gain function
		void galerkin(VectorXd &h_diff, MatrixXd &K);

		// Compute basis functions on SO(3) for each particle
		void compute_basis_SO3(const MatrixXd &particles, MatrixXd &Phi);

		// Compute gradient of basis functions on SO(3) for each particle
		void compute_basisGrad_SO3(const MatrixXd &particles, Tensor3Xd &gradPhi);

		virtual std::ostream& message(std::ostream &out) const override;
	};

} // End of attitude_estimation namespace


#endif

