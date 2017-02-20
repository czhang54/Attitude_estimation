
/* Implementation of the multiplicative EKF (MEKF) algorithm */

#include <iostream>
#include <cmath>
#include <random>

#include <Eigen/Dense>

#include "World.h"
#include "Target.h"
#include "Sensor.h"
#include "Filter.h"
#include "MultiplicativeEKF.h"
#include "Lie_group.h"


namespace attitude_estimation{


	void MultiplicativeEKF::update(int TI, double dt, std::default_random_engine &generator){

		std::cout << "MEKF ";

		if (TI == 0){performance(TI);}

		/* PREDICTION */
		VectorXd target_velocity = world->get_target()->get_velocity().col(TI);
		quaternion predict_mean = dq(estimates.col(TI), target_velocity*dt);
		MatrixXd Phi = identity(Q_d.rows()) - skew(target_velocity)*dt;
		MatrixXd predict_cov = Phi*covariance*(Phi.transpose()) + Q_d*dt;

		/* MEASUREMENT UPDATE */
		// Compute Jacobian matrix H and innovation vector I
		MatrixXd H = MatrixXd::Zero(dim_sensor, dim_space);
		VectorXd I = VectorXd::Zero(dim_sensor);
		int count(0);
		for (SensorBase *s: world->get_sensor()){
			VectorXd measurement = s->get_measurements().col(TI+1);
			int dim = measurement.size();
			H.block(count, 0, dim, dim_space) = s->jacobian(predict_mean);
			I.segment(count, dim) = s->get_measurements().col(TI+1) - s->model(predict_mean);
			count += dim;
		}
		// Compute gain matrix K
		MatrixXd S = H*predict_cov*(H.transpose()) + Q_v;
		MatrixXd K = predict_cov*(H.transpose())*(S.inverse());

		// Update mean and covariance of MEKF;
		quaternion update_mean = quaternion_product(predict_mean, MRP2q(K*I)); 
		covariance = (identity(Q_d.rows()) - K*H)*predict_cov;
		estimates.col(TI+1) = update_mean;

		performance(TI+1);

	}

	std::ostream& MultiplicativeEKF::message(std::ostream &out) const {
		out << "Multiplicative EKF will be implemented" << '\n';
		return out;
	}


} // End of attitude_estimation namespace


