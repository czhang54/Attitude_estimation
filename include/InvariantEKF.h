
/* Define the invariant EKF (IEKF) algorithm */

#ifndef INVARIANTEKF
#define INVARIANTEKF

#include <iostream>

#include <Eigen/Dense>

#include "Filter.h"


namespace attitude_estimation{


	class InvariantEKF: public KalmanFilterBase{

	public:

		// Constructor
		InvariantEKF(const VectorXd &IC_mean, const VectorXd &IC_std)
			: KalmanFilterBase(IC_mean, IC_std) {}

		// No need to re-define initialize() method for IEKF. Use KalmanFilter::initialize().

		// Run IEKF for one iteration
		virtual void update(int TI, double dt, std::default_random_engine &generator) override;

		virtual std::ostream& message(std::ostream &out) const override;

	};


} // End of attitude_estimation namespace



#endif