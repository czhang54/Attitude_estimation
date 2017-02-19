
/* Implementation of the multiplicative EKF (MEKF) algorithm */

#include <iostream>
#include <cmath>
#include <random>

#include <Eigen/Dense>

#include "World.h"
#include "Target.h"
#include "Sensor.h"
#include "Filter.h"
#include "InvariantEKF.h"
#include "Lie_group.h"


namespace attitude_estimation{


	void InvariantEKF::update(int TI, double dt, std::default_random_engine &generator){

		std::cout << "IEKF ";

		if (TI == 0){performance(TI);}

		/* PREDICTION */
		VectorXd target_velocity = world->get_target()->get_velocity().col(TI);
		quaternion predict_mean = dq(estimates.col(TI), target_velocity*dt);
		RotationMatrix predict_mean_R = q2R(predict_mean);
		MatrixXd predict_cov = covariance + Q_d*dt;

		/* MEASUREMENT UPDATE */
		// Compute Jacobian matrix H and innovation vector I
		MatrixXd H = MatrixXd::Zero(dim_sensor, dim_space);
		VectorXd I = VectorXd::Zero(dim_sensor);
		int count(0);
		for (SensorBase *s: world->get_sensor()){
			VectorXd measurement = s->get_measurements().col(TI+1);
			int dim = measurement.size();
			H.block(count, 0, dim, dim_space) = predict_mean_R*(s->jacobian(predict_mean))*(predict_mean_R.transpose());
			I.segment(count, dim) = predict_mean_R*(s->get_measurements().col(TI+1) - s->model(predict_mean));
			// cout << s->get_measurements().col(TI+1).transpose() << '\n';
			count += dim;
		}

		// Compute gain matrix K
		MatrixXd S = H*predict_cov*(H.transpose()) + Q_v;
		MatrixXd K = predict_cov*(H.transpose())*(S.inverse());

		// Update mean and covariance of IEKF;
		quaternion update_mean = dq(predict_mean, K*I, "global_frame");
		covariance = (identity(Q_d.rows()) - K*H)*predict_cov;
		estimates.col(TI+1) = update_mean;

		performance(TI+1);

	}

	std::ostream& InvariantEKF::message(std::ostream &out) const {
		out << "Invariant EKF will be implemented" << '\n';
		return out;
	}


} // End of attitude_estimation namespace


