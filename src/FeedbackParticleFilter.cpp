
/* Implementation of the feedback particle filter (FPF) algorithm */

#include <iostream>
#include <cmath>
#include <random>

#include <Eigen/Dense>

#include "World.h"
#include "Target.h"
#include "Sensor.h"
#include "Filter.h"
#include "FeedbackParticleFilter.h"
#include "Lie_group.h"


namespace attitude_estimation{

	// Run FPF algorithm for one iteration
	void FeedbackParticleFilter::update(int TI, double dt, std::default_random_engine &generator){

		std::cout << "FPF ";

		if (TI==0) {performance(TI);}

		VectorXd target_velocity = world->get_target()->get_velocity().col(TI);
		VectorXd process_noise_std = world->get_target()->get_process_noise();

		double dt_subdivide = dt/(static_cast<double>(TI_subdivide_));

		if (TI >= TI_subdivide_){num_subdivide_ = 1;}

		for (int n=0; n<num_subdivide_; ++n){

			// Compute control in Lie algebra for each particle
			MatrixXd control = MatrixXd::Zero(dim_space, N);

			for (SensorBase *s: world->get_sensor()){
				// Iterate over all sensors
				VectorXd sensor_noise_cov = s->get_sensor_noise().array().square();
				VectorXd measurement = s->get_measurements().col(TI);
				MatrixXd h = MatrixXd::Zero(measurement.rows(), N);
				for (int i=0; i<N; ++i){
					h.col(i) = s->model(particles.col(i));
				}
				for (int j=0; j<measurement.rows(); ++j){
					// For each sensor measurement, compute innovation I and gain K at EACH particle
					VectorXd h_diff = h.row(j).array() - h.row(j).mean();
					VectorXd I = measurement(j) - 0.5*(h.row(j).array() + h.row(j).mean());
					MatrixXd K = MatrixXd::Zero(dim_space, N);

					// Compute gain values using a Galerkin method (standard for numerically solving a PDE)
					galerkin(h_diff, K); 

					for (int n=0; n<dim_space; ++n){
						RowVectorXd KI = (K.row(n).array())*(I.transpose().array())/(sensor_noise_cov(j)*num_subdivide_);
						control.row(n) = control.row(n) + KI;
					}
				}
			}
			for (int i=0; i<N; ++i){
				// Add process noise for each particle
				VectorXd process_noise = VectorXd::Constant(dim_space, 0);
				for (int d=0; d<dim_space; ++d){
					std::normal_distribution<double> distribution(0, process_noise_std(d)*sqrt(dt_subdivide));
					process_noise(d) = distribution(generator);
				}
				// Propagate each particle
				particles.col(i) = dq(particles.col(i), control.col(i) + target_velocity*dt_subdivide + process_noise );
			}
		}

		estimates.col(TI+1) = quaternion_mean(particles); // Compute average of quaternion particles

		performance(TI+1);
	}


	typedef Matrix<MatrixXd, Dynamic, 1> Tensor3Xd; // 3d array (tensor)

	// Galerkin method to solve gain function
	void FeedbackParticleFilter::galerkin(VectorXd &h_diff, MatrixXd &K){

		const int L = 9; // 9 basis functions defined on SO(3)

		MatrixXd  Phi(L, N); // Basis function evaluated at each particle
		Tensor3Xd gradPhi(dim_space); // Gradient of basis function evaluated at each particle
		for (int d=0; d<dim_space; ++d){
			gradPhi(d) = MatrixXd::Zero(L, N);
		}

		// Evaluate basis functions and their gradient on SO(3)
		compute_basis_SO3(particles, Phi);
		compute_basisGrad_SO3(particles, gradPhi);

		MatrixXd A = MatrixXd::Zero(L, L); 
		VectorXd b = VectorXd::Zero(L);	
		VectorXd k = VectorXd::Zero(L);

		// Assemble A matrix and b vector
		for (int l=0; l<L; l++){
			b(l) = h_diff.dot(Phi.row(l))/(static_cast<double>(N));
			for (int m=l; m<L; ++m){
				double sum = 0.0;
				for (int d=0; d<dim_space; ++d){
					sum += (gradPhi(d).row(l)).dot(gradPhi(d).row(m));
				}
				A(l,m) = sum/(static_cast<double>(N));
				A(m,l) = A(l,m);
			}
		}

		// Solve matrix equation Ak = b for k
		k = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);

		for (int d=0; d<dim_space; ++d){
			K.row(d) = (k.transpose())*gradPhi(d);
		}

	}

	// Compute basis functions on SO(3) for each particle
	void FeedbackParticleFilter::compute_basis_SO3(const MatrixXd &particles, MatrixXd &Phi){

		Phi.row(0) = 2*(particles.row(0).array().square() + particles.row(3).array().square()) - 1; // OK with Eigen
		Phi.row(1) = 2*(particles.row(0).array()*particles.row(2).array() + particles.row(1).array()*particles.row(3).array());
		Phi.row(2) = 2*(particles.row(0).array()*particles.row(1).array() - particles.row(2).array()*particles.row(3).array());
		Phi.row(3) = 2*(-particles.row(0).array()*particles.row(2).array() + particles.row(1).array()*particles.row(3).array());
		Phi.row(4) = 2*(particles.row(0).array()*particles.row(1).array() + particles.row(2).array()*particles.row(3).array());
		Phi.row(5) = 2*particles.row(0).array()*particles.row(3).array();
		Phi.row(6) = particles.row(0).array().square() - particles.row(3).array().square();
		Phi.row(7) = 2*particles.row(1).array()*particles.row(2).array();
		Phi.row(8) = particles.row(1).array().square() - particles.row(2).array().square();
	}
	
	// Compute gradient of basis functions on SO(3) for each particle
	void FeedbackParticleFilter::compute_basisGrad_SO3(const MatrixXd &particles, Tensor3Xd &gradPhi){

		gradPhi(0).row(0) = -2*(particles.row(0).array()*particles.row(1).array() + particles.row(2).array()*particles.row(3).array());
		gradPhi(0).row(1) = 2*(particles.row(0).array()*particles.row(3).array() - particles.row(1).array()*particles.row(2).array());
		gradPhi(0).row(2) = 2*(particles.row(0).array().square() + particles.row(2).array().square()) - 1;
		gradPhi(0).row(3) = VectorXd::Zero(N);
		gradPhi(0).row(4) = 2*(particles.row(0).array().square() + particles.row(3).array().square()) - 1;
		gradPhi(0).row(5) = -particles.row(0).array()*particles.row(2).array() - particles.row(1).array()*particles.row(3).array();
		gradPhi(0).row(6) = -particles.row(0).array()*particles.row(1).array() + particles.row(2).array()*particles.row(3).array();
		gradPhi(0).row(7) = particles.row(0).array()*particles.row(2).array() + particles.row(1).array()*particles.row(3).array();
		gradPhi(0).row(8) = particles.row(0).array()*particles.row(1).array() - particles.row(2).array()*particles.row(3).array();

		gradPhi(1).row(0) = 2*(-particles.row(0).array()*particles.row(2).array() + particles.row(1).array()*particles.row(3).array());
		gradPhi(1).row(1) = 2*(particles.row(0).array().square() + particles.row(1).array().square()) - 1;
		gradPhi(1).row(2) = -2*(particles.row(0).array()*particles.row(3).array() + particles.row(1).array()*particles.row(2).array());
		gradPhi(1).row(3) = -2*(particles.row(0).array().square() + particles.row(3).array().square()) + 1;
		gradPhi(1).row(4) = VectorXd::Zero(N);	
		gradPhi(1).row(5) = particles.row(0).array()*particles.row(1).array() - particles.row(2).array()*particles.row(3).array();
		gradPhi(1).row(6) = -particles.row(0).array()*particles.row(2).array() - particles.row(1).array()*particles.row(3).array();
		gradPhi(1).row(7) = particles.row(0).array()*particles.row(1).array() - particles.row(2).array()*particles.row(3).array();
		gradPhi(1).row(8) = -particles.row(0).array()*particles.row(2).array() - particles.row(1).array()*particles.row(3).array();

		gradPhi(2).row(0) = VectorXd::Zero(N);
		gradPhi(2).row(1) = VectorXd::Zero(N);
		gradPhi(2).row(2) = VectorXd::Zero(N);
		gradPhi(2).row(3) = 2*(particles.row(0).array()*particles.row(1).array() + particles.row(2).array()*particles.row(3).array());
		gradPhi(2).row(4) = 2*(particles.row(0).array()*particles.row(2).array() - particles.row(1).array()*particles.row(3).array());
		gradPhi(2).row(5) = particles.row(0).array().square() - particles.row(3).array().square();
		gradPhi(2).row(6) = -2*particles.row(0).array()*particles.row(3).array();
		gradPhi(2).row(7) = particles.row(2).array().square() - particles.row(1).array().square();
		gradPhi(2).row(8) = 2*particles.row(1).array()*particles.row(2).array();

	}


	std::ostream& FeedbackParticleFilter::message(std::ostream &out) const {
		out << "Feedback particle filter with " << gain_solver_ << " gain solver will be implemented" << '\n';
		return out;
	}


} // End of attitude_estimation namespace





