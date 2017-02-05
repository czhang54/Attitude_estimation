#include <iostream>
#include <cmath>
#include <random>

#include <Eigen/Dense>

#include "../include/World.h"
#include "../include/Target.h"
#include "../include/Sensor.h"
#include "../include/Filter.h"
#include "../include/Lie_group.h"

using namespace std;
using namespace Eigen;


/* ########## Filter (base class for all filters) ########## */

void Filter::initialize(default_random_engine &generator) {
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

void Filter::update(int TI, double dt, default_random_engine &generator){
	estimates.col(TI+1) = estimates.col(TI);
}

void Filter::performance(int TI){
	quaternion target_state = world->get_target()->get_state().col(TI);
	quaternion estimate_error = quaternion_product(inverse_quaternion(estimates.col(TI)), target_state);
	angle_error(TI) = 2*acos(abs(estimate_error(0)))*180/3.14;
	// cout << "Estimate error = " << '\n';
	cout << "angle error = " << 2*acos(abs(estimate_error(0)))*180/3.14 << '\n';
}

ostream& operator<<(ostream &out, const Filter *f){
	return f->message(out);
}

ostream& Filter::message(ostream &out) const {
	out << "A filter will be implemented" << '\n';
	return out;
}


/* ########## KalmanFilter (base class for all Kalman-type filters) ########## */
void KalmanFilter::initialize(default_random_engine &generator){

	Filter::initialize(generator); // Note the scope operator

	VectorXd IC_cov = IC_std.array().square(); // square().asDiagonal() is not allowed!
	covariance = IC_cov.asDiagonal(); // Initialize filter covariance, updated using MEKF::update

	// Construct noise matrices Q_d, Q_v
	VectorXd process_noise_cov = world->get_target()->get_process_noise().array().square();
	Q_d = process_noise_cov.asDiagonal();

	// Simpler way: dim_sum = 3*world.get_sensor().size() if only vector measurements are used.
	int dim_sum = 0;
	Q_v = MatrixXd::Zero(dim_sensor, dim_sensor);
	for (int i=0; i<world->get_sensor().size(); ++i){
		VectorXd sensor_noise_cov = world->get_sensor()[i]->get_sensor_noise().array().square();
		int dim = sensor_noise_cov.size();
		Q_v.block(dim_sum, dim_sum, dim, dim) = sensor_noise_cov.asDiagonal();
		dim_sum += dim;
	}
	// cout << Q_d << '\n';
	// cout << Q_v << '\n';
}


/* ########## MEKF ########## */

void MEKF::update(int TI, double dt, default_random_engine &generator){

	cout << "MEKF ";

	if (TI == 0){performance(TI);}

	/* PROPAGATION */
	VectorXd target_velocity = world->get_target()->get_velocity().col(TI);
	quaternion pred_mean = dq(estimates.col(TI), target_velocity*dt);
	MatrixXd Phi = identity(Q_d.rows()) - skew(target_velocity)*dt;
	MatrixXd pred_cov = Phi*covariance*(Phi.transpose()) + Q_d*dt;

	/* MEASUREMENT UPDATE */
	// Compute Jacobian matrix and innovation
	MatrixXd H = MatrixXd::Zero(dim_sensor, dim_space);
	VectorXd I = VectorXd::Zero(dim_sensor);
	int count(0);
	for (Sensor *s: world->get_sensor()){
		VectorXd measurement = s->get_measurements().col(TI+1);
		int dim = measurement.size();
		H.block(count, 0, dim, dim_space) = s->jacobian(pred_mean);
		I.segment(count, dim) = s->get_measurements().col(TI+1) - s->model(pred_mean);
		// cout << s->get_measurements().col(TI+1).transpose() << '\n';
		// cout << s->model(pred_mean).transpose() << '\n';
		// cout << I.segment(count, dim).transpose() << '\n';
		count += dim;
	}
	// cout << I.transpose() << '\n';
	// Compute gain matrix
	MatrixXd S = H*pred_cov*(H.transpose()) + Q_v;
	MatrixXd K = pred_cov*(H.transpose())*(S.inverse());

	// Update mean and covariance;
	quaternion update_mean = quaternion_product(pred_mean, MRP2q(K*I)); 
	// cout << update_mean.transpose() << '\n';
	covariance = (identity(Q_d.rows()) - K*H)*pred_cov;
	estimates.col(TI+1) = update_mean;

	performance(TI+1);

}

ostream& MEKF::message(ostream &out) const {
	out << "MEKF will be implemented" << '\n';
	return out;
}


/* ########## IEKF ########## */

void IEKF::update(int TI, double dt, default_random_engine &generator){

	cout << "IEKF ";

	if (TI == 0){performance(TI);}

	/* PROPAGATION */
	VectorXd target_velocity = world->get_target()->get_velocity().col(TI);
	quaternion pred_mean = dq(estimates.col(TI), target_velocity*dt);
	RotationMatrix pred_mean_R = q2R(pred_mean);
	MatrixXd pred_cov = covariance + Q_d*dt;

	/* MEASUREMENT UPDATE */
	// Compute Jacobian matrix and innovation
	MatrixXd H = MatrixXd::Zero(dim_sensor, dim_space);
	VectorXd I = VectorXd::Zero(dim_sensor);
	int count(0);
	for (Sensor *s: world->get_sensor()){
		VectorXd measurement = s->get_measurements().col(TI+1);
		int dim = measurement.size();
		H.block(count, 0, dim, dim_space) = pred_mean_R*(s->jacobian(pred_mean))*(pred_mean_R.transpose());
		I.segment(count, dim) = pred_mean_R*(s->get_measurements().col(TI+1) - s->model(pred_mean));
		// cout << s->get_measurements().col(TI+1).transpose() << '\n';
		count += dim;
	}

	// Compute gain matrix
	MatrixXd S = H*pred_cov*(H.transpose()) + Q_v;
	MatrixXd K = pred_cov*(H.transpose())*(S.inverse());

	// Update mean and covariance;
	quaternion update_mean = dq(pred_mean, K*I, "global_frame");
	covariance = (identity(Q_d.rows()) - K*H)*pred_cov;
	estimates.col(TI+1) = update_mean;

	performance(TI+1);

}

ostream& IEKF::message(ostream &out) const {
	out << "IEKF will be implemented" << '\n';
	return out;
}


/* ########## ParticleFilterBase (base class for all particle-based filters) ########## */

void ParticleFilterBase::initialize(default_random_engine &generator){

	Filter::initialize(generator);

	// Generate initial particles
	particles = MatrixXd::Zero(dim_filter, N);

	ParticleFilterBase::initialize_gaussian(generator); // Scope operator seems optional here
	// cout << "Initial particles generated" << '\n';

	sensor_noise_cov = VectorXd::Zero(dim_sensor);
	int count = 0;
	for (Sensor *s: world->get_sensor()){
		int dim = s->get_sensor_noise().size();
		sensor_noise_cov.segment(count, dim) = s->get_sensor_noise().array().square();
		count += dim;
	}
}

void ParticleFilterBase::initialize_gaussian(default_random_engine &generator){
	// First generate samples in Lie algebra (LA), and then project onto Lie group using exponential map

	// default_random_engine generator;
	// generator.seed(static_cast<unsigned int>(std::time(0))); 

	for (int i=0; i<N; i++){	
		VectorXd particle_LA(dim_space);
		for (int d=0; d<dim_space; d++){
			normal_distribution<double> normal(0, IC_std(d));
			particle_LA(d) = normal(generator);
		}
		particles.col(i) = dq(IC_mean, particle_LA);
	}
}


ostream& ParticleFilterBase::message(ostream &out) const {
		out << "Particle filters will be implemented" << '\n';
		return out;
	}


/* ########## ParticleFilter (Classical particle filter using sequential imprtance sampling resampling) ########## */

void ParticleFilter::update(int TI, double dt, default_random_engine &generator){

	cout << "Particle filter ";

	if (TI == 0){performance(TI);}

	/* PROPAGATION */
	VectorXd target_velocity = world->get_target()->get_velocity().col(TI);
	VectorXd process_noise_std = world->get_target()->get_process_noise();
	// Add process noise
	VectorXd process_noise = VectorXd::Constant(dim_space, 0);
	for (int d=0; d<dim_space; ++d){
		normal_distribution<double> distribution(0, process_noise_std(d)*sqrt(dt));
		process_noise(d) = distribution(generator);
	}
	for (int i=0; i<N; ++i){
		particles.col(i) = dq(particles.col(i), target_velocity*dt + process_noise);
	}

	/* MEASUREMENT UPDATE */
	// Compute weights
	VectorXd w = VectorXd::Zero(N);
	for (Sensor *s: world->get_sensor()){
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
	resampling(weights);

	// Diffuse particles
	// default_random_engine generator;
	// generator.seed(static_cast<unsigned int>(std::time(0))); 

	for (int i=0; i<N; i++){	
		VectorXd particle_LA(dim_space);
		for (int d=0; d<dim_space; d++){
			normal_distribution<double> normal(0, diffuse_kernel);
			particle_LA(d) = normal(generator);
		}
		particles.col(i) = dq(particles.col(i), particle_LA);
	}

	estimates.col(TI+1) = quaternion_mean(particles);

	performance(TI+1);

}


void ParticleFilter::resampling(VectorXd &weights){
	// Resampling with replacement
	MatrixXd resampled_particles = MatrixXd::Zero(dim_filter, N);

	default_random_engine generator;
	generator.seed(static_cast<unsigned int>(std::time(0)));
	uniform_real_distribution<double> uniform(0, 1/(static_cast<double>(N)));
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


ostream& ParticleFilter::message(ostream &out) const {
	out << "Particle filter will be implemented" << '\n';
	return out;
}


/* ########## FPF (Feedback particle filter with multiple gain solver options) ########## */

void FPF::update(int TI, double dt, default_random_engine &generator){

	cout << "FPF ";

	if (TI==0) {performance(TI);}

	VectorXd target_velocity = world->get_target()->get_velocity().col(TI);
	VectorXd process_noise_std = world->get_target()->get_process_noise();

	double dt_subdivide = dt/(static_cast<double>(TI_subdivide));

	if (TI >= TI_subdivide){num_subdivide = 1;}

	for (int n=0; n<num_subdivide; ++n){
		// Compute control force in Lie algebra for each particle
		MatrixXd control = MatrixXd::Zero(dim_space, N);
		for (Sensor *s: world->get_sensor()){
			VectorXd sensor_noise_cov = s->get_sensor_noise().array().square();
			VectorXd measurement = s->get_measurements().col(TI);
			MatrixXd h = MatrixXd::Zero(measurement.rows(), N);
			for (int i=0; i<N; ++i){
				h.col(i) = s->model(particles.col(i));
			}
			for (int j=0; j<measurement.rows(); ++j){
				VectorXd h_diff = h.row(j).array() - h.row(j).mean();
				VectorXd I = measurement(j) - 0.5*(h.row(j).array() + h.row(j).mean());
				MatrixXd K = MatrixXd::Zero(dim_space, N);
				// Compute gain function
				galerkin(h_diff, K);
				for (int n=0; n<dim_space; ++n){
					RowVectorXd KI = (K.row(n).array())*(I.transpose().array())/(sensor_noise_cov(j)*num_subdivide);
					control.row(n) = control.row(n) + KI;
				}
			}
		}
		for (int i=0; i<N; ++i){
			// Add process noise
			VectorXd process_noise = VectorXd::Constant(dim_space, 0);
			for (int d=0; d<dim_space; ++d){
				normal_distribution<double> distribution(0, process_noise_std(d)*sqrt(dt_subdivide));
				process_noise(d) = distribution(generator);
			}
			particles.col(i) = dq(particles.col(i), control.col(i) + target_velocity*dt_subdivide + process_noise );
		}
	}

	estimates.col(TI+1) = quaternion_mean(particles);

	performance(TI+1);
}

typedef Matrix<MatrixXd, Dynamic, 1> Tensor3Xd; // 3d array (tensor)

void FPF::galerkin(VectorXd &h_diff, MatrixXd &K){

	const int L = 9; // Number of basis functions

	MatrixXd  Phi(L, N);
	Tensor3Xd gradPhi(dim_space);
	// Need to initialize gradPhi in full size
	for (int d=0; d<dim_space; ++d){
		gradPhi(d) = MatrixXd::Zero(L, N);
	}

	// Evaluate basis functions and their gradient
	compute_basis_SO3(particles, Phi);
	compute_basisGrad_SO3(particles, gradPhi);
	// Solve coefficients
	MatrixXd A = MatrixXd::Zero(L, L);
	VectorXd b = VectorXd::Zero(L);	
	VectorXd k = VectorXd::Zero(L);

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

	k = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);

	for (int d=0; d<dim_space; ++d){
		K.row(d) = (k.transpose())*gradPhi(d);
	}

}

void FPF::compute_basis_SO3(const MatrixXd &particles, MatrixXd &Phi){

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

void FPF::compute_basisGrad_SO3(const MatrixXd &particles, Tensor3Xd &gradPhi){

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


ostream& FPF::message(ostream &out) const {
	out << "FPF with " << gain_solver << " gain solver will be implemented" << '\n';
	return out;
}


