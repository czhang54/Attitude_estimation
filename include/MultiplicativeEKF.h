
/* Define the multiplicative EKF (MEKF) algorithm */

#ifndef MULTIPLICATIVEEKF
#define MULTIPLICATIVEEKF

#include <iostream>

#include <Eigen/Dense>

#include "Filter.h"


namespace attitude_estimation{


	class MultiplicativeEKF: public KalmanFilterBase{

	public:

		// Constructor
		MultiplicativeEKF(const VectorXd &IC_mean, const VectorXd &IC_std)
			: KalmanFilterBase(IC_mean, IC_std) {}

		// No need to re-define initialize() method for MEKF. Use KalmanFilter::initialize().

		// Run MEKF for one iteration
		virtual void update(int TI, double dt, std::default_random_engine &generator) override;

		virtual std::ostream& message(std::ostream &out) const override;

	};


} // End of attitude_estimation namespace



#endif