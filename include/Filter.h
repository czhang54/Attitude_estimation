#ifndef FILTER
#define FILTER

#include <iostream>

#include <Eigen/Dense>

// using namespace std;
// using namespace Eigen;

namespace Attitude_estimation{

	using namespace Eigen;

	class World; // Forward declaration

	/* ########## FilterBase (base class for all filters) ########## */
	class FilterBase
	{
	protected:

		World *world; // A pointer to the World, set when the filter is initialized
		int dim_filter; // Dimension of filter state, e.g. dim_filter=4 if quaternion is used
		int dim_space; // Dimension of space, e.g. dim_space=3 for SO(3)
		int dim_sensor; // Total dimension of all sensor measurements
		MatrixXd estimates; // Mean of filter estimate at all time instants
		RowVectorXd angle_error; // Rotation angle estimation error at all time instants
		VectorXd IC_mean; // Mean of initial filter estimate
		VectorXd IC_std; // Standard deviation in all directions of initial filter estimate
		
	public:

		FilterBase(const VectorXd &IC_mean, const VectorXd &IC_std) 
			: IC_mean(IC_mean), IC_std(IC_std) {}

		// Access filter estimates
		MatrixXd& get_estimates();

		// Initialize a filter. 
		virtual void initialize(std::default_random_engine &generator); 

		// Update filter estimates (execute filter algorithm)
		virtual void update(int TI, double dt, std::default_random_engine &generator);

		// Evaluate estimation error according to some performance measure
		void performance(int TI);

		// Overload << operator
		friend std::ostream& operator<<(std::ostream &out, const FilterBase *f);
		virtual std::ostream& message(std::ostream &out) const;

		friend World;

	};


	/* ########## Kalman filters ########## */
	
	/* Base class for Kalman filters */
	class KalmanFilterBase: public FilterBase
	{

	protected:
		MatrixXd Q_d; // Process noise matrix
		MatrixXd Q_v; // Sensor noise matrix
		MatrixXd covariance; // Filter covariance, updated at each iteration

	public:
		KalmanFilterBase(const VectorXd &IC_mean, const VectorXd &IC_std)
			: FilterBase(IC_mean, IC_std) {}

		// Initialize a Kalman filter
		virtual void initialize(std::default_random_engine &generator) override;

		// No need to define update() method for KalmanFilter, use override version in MEKF, IEKF etc.

	};

	/* Multiplicative EKF (MEKF) */
	class MEKF: public KalmanFilterBase
	{

	public:

		MEKF(const VectorXd &IC_mean, const VectorXd &IC_std)
			: KalmanFilterBase(IC_mean, IC_std) {}

		// No need to re-define initialize() method for MEKF. Use KalmanFilter::initialize().

		// Algorithm fo MEKF
		virtual void update(int TI, double dt, std::default_random_engine &generator) override;

		virtual std::ostream& message(std::ostream &out) const override;

	};

	/* Invariant EKF (IEKF) */
	class IEKF: public KalmanFilterBase
	{

	public: 

		IEKF(const VectorXd &IC_mean, const VectorXd &IC_std)
			: KalmanFilterBase(IC_mean, IC_std) {}

		// IEKF algorithm
		virtual void update(int TI, double dt, std::default_random_engine &generator) override;

		virtual std::ostream& message(std::ostream &out) const override;

	};


	/* ########## Particle filters ########## */
	
	/* Base class of particle filters */
	class ParticleFilterBase: public FilterBase{

	protected:
		int N; // Number of particles
		MatrixXd particles; // Store current (quaternion) particles, updated after every iteration
		VectorXd sensor_noise_cov; // Noise covariance of ALL sensors 

	public:

		ParticleFilterBase(const VectorXd &IC_mean, const VectorXd &IC_std, const int num_particles)
			: FilterBase(IC_mean, IC_std), N(num_particles) {}

		// Initialize a particle filter, generate initial particles
		virtual void initialize(std::default_random_engine &generator) override;

		// Generate initial particles from a Gaussian distribution
		void initialize_gaussian(std::default_random_engine &generator);

		// No need to define update() method for ParticleFilter, use override version in BPF and FPF.

		virtual std::ostream& message(std::ostream &out) const override;

	};


	/* Particle Filter (Classical particle filter using sequential imprtance sampling resampling) */
	class ParticleFilter: public ParticleFilterBase
	{

		double diffuse_kernel; // Noise parameter to diffuse particles after resampling

	public:

		ParticleFilter(const VectorXd &IC_mean, const VectorXd &IC_std, const int num_particles, const double diffuse_kernel)
			: ParticleFilterBase(IC_mean, IC_std, num_particles), diffuse_kernel(diffuse_kernel) {}

		// Particle filter algorithm
		virtual void update(int TI, double dt, std::default_random_engine &generator) override;

		// Resampling procedure
		void resampling(VectorXd &weights, std::default_random_engine &generator);
	 
		virtual std::ostream& message(std::ostream &out) const override;

	};


	typedef Matrix<MatrixXd, Dynamic, 1> Tensor3Xd; // 3d array (tensor)

	/* Feedback particle filter (FPF) */
	class FPF: public ParticleFilterBase
	{
		std::string gain_solver; // Name of gain solver
		int TI_subdivide; // Maximum number of filter steps that further sub-divided
		int num_subdivide; // Number of subdivided steps within each normal step
	 
	public:

		FPF(const VectorXd &IC_mean, const VectorXd &IC_std, const int num_particles, std::string gain_solver, int TI_subdivide=0, int num_subdivide=1)
			: ParticleFilterBase(IC_mean, IC_std, num_particles), gain_solver(gain_solver), TI_subdivide(TI_subdivide), num_subdivide(num_subdivide) {}

		// FPF algorithm
		virtual void update(int TI, double dt, std::default_random_engine &generator) override;

		// Galerkin method to solve gain function
		void galerkin(VectorXd &h_diff, MatrixXd &K);

		// Compute basis functions on SO(3) for Galerkin
		void compute_basis_SO3(const MatrixXd &particles, MatrixXd &Phi);

		// Compute gradient of basis functions on SO(3) for Galerkin
		void compute_basisGrad_SO3(const MatrixXd &particles, Tensor3Xd &gradPhi);

		virtual std::ostream& message(std::ostream &out) const override;


	};


}


















#endif
