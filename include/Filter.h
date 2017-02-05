#ifndef FILTER
#define FILTER

#include <iostream>

#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

class World;

class Filter
{
protected:

	World *world;
	int dim_filter;
	int dim_space;
	int dim_sensor; // Total dimension of all sensor measurements
	MatrixXd estimates;
	RowVectorXd angle_error;
	VectorXd IC_mean;
	VectorXd IC_std;
	
public:

	Filter(const VectorXd &IC_mean, const VectorXd &IC_std) 
		: IC_mean(IC_mean), IC_std(IC_std) {}

	MatrixXd& get_estimates();

	virtual void initialize(default_random_engine &generator); // Initialization is different for EKF, UKF and particle filters

	virtual void update(int TI, double dt, default_random_engine &generator);

	void performance(int TI);

	friend ostream& operator<<(ostream &out, const Filter *f);

	virtual ostream& message(ostream &out) const;

	friend World;

};


class KalmanFilter: public Filter
{

protected:
	MatrixXd Q_d; // Process noise matrix
	MatrixXd Q_v; // Sensor noise matrix
	MatrixXd covariance; // Filter covariance at CURRENT time instant

public:
	KalmanFilter(const VectorXd &IC_mean, const VectorXd &IC_std)
		: Filter(IC_mean, IC_std) {}

	virtual void initialize(default_random_engine &generator) override;

	// No need to define update() method for KalmanFilter, use override version in MEKF, IEKF etc.

};


class MEKF: public KalmanFilter
{

public:

	MEKF(const VectorXd &IC_mean, const VectorXd &IC_std)
		: KalmanFilter(IC_mean, IC_std) {}

	// No need to re-define initialize() method for MEKF. Will use KalmanFilter::initialize().

	virtual void update(int TI, double dt, default_random_engine &generator) override;

	virtual ostream& message(ostream &out) const override;

};


class IEKF: public KalmanFilter
{

public: 

	IEKF(const VectorXd &IC_mean, const VectorXd &IC_std)
		: KalmanFilter(IC_mean, IC_std) {}

	virtual void update(int TI, double dt, default_random_engine &generator) override;

	virtual ostream& message(ostream &out) const override;

};


class ParticleFilterBase: public Filter{

protected:
	int N; // Number of particles
	MatrixXd particles; // Store current (quaternion) particles, updated after every iteration
	VectorXd sensor_noise_cov; // Noise covariance of ALL sensors 

public:

	ParticleFilterBase(const VectorXd &IC_mean, const VectorXd &IC_std, const int num_particles)
		: Filter(IC_mean, IC_std), N(num_particles) {}

	virtual void initialize(default_random_engine &generator) override;

	void initialize_gaussian(default_random_engine &generator);

	// void initialize_uniform();

	// No need to define update() method for ParticleFilter, use override version in BPF, FPF etc.

	virtual ostream& message(ostream &out) const override;

};


class ParticleFilter: public ParticleFilterBase
{

	double diffuse_kernel;

public:

	ParticleFilter(const VectorXd &IC_mean, const VectorXd &IC_std, const int num_particles, const double diffuse_kernel)
		: ParticleFilterBase(IC_mean, IC_std, num_particles), diffuse_kernel(diffuse_kernel) {}

	virtual void update(int TI, double dt, default_random_engine &generator) override;

	void resampling(VectorXd &weights);
 
	virtual ostream& message(ostream &out) const override;

};


typedef Matrix<MatrixXd, Dynamic, 1> Tensor3Xd; // 3d array (tensor)

class FPF: public ParticleFilterBase
{
	string gain_solver;
	int TI_subdivide;
	int num_subdivide;

public:

	FPF(const VectorXd &IC_mean, const VectorXd &IC_std, const int num_particles, string gain_solver, int TI_subdivide=0, int num_subdivide=1)
		: ParticleFilterBase(IC_mean, IC_std, num_particles), gain_solver(gain_solver), TI_subdivide(TI_subdivide), num_subdivide(num_subdivide) {}

	virtual void update(int TI, double dt, default_random_engine &generator) override;

	void galerkin(VectorXd &h_diff, MatrixXd &K);

	void compute_basis_SO3(const MatrixXd &particles, MatrixXd &Phi);

	void compute_basisGrad_SO3(const MatrixXd &particles, Tensor3Xd &gradPhi);

	virtual ostream& message(ostream &out) const override;


};





















#endif