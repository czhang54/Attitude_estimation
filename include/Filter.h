
/* This header defines:
   (1) base class of all filters, 
   (2) base class of all Kalman filters, and 
   (3) base class of all particle filters. */

#ifndef FILTER
#define FILTER

#include <iostream>

#include <Eigen/Dense>

// using namespace std;
// using namespace Eigen;

namespace attitude_estimation{

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

		// Constructor
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


	/* ########## Kalman filter base class ########## */
	class KalmanFilterBase: public FilterBase
	{

	protected:
		MatrixXd Q_d; // Process noise matrix
		MatrixXd Q_v; // Sensor noise matrix
		MatrixXd covariance; // Filter covariance, updated at each iteration

	public:

		// Constructor
		KalmanFilterBase(const VectorXd &IC_mean, const VectorXd &IC_std)
			: FilterBase(IC_mean, IC_std) {}

		// Initialize a Kalman filter
		virtual void initialize(std::default_random_engine &generator) override;

		// No need to define update() method for KalmanFilter, use override version in MEKF, IEKF etc.

	};



	/* ########## Particle filter base class ########## */
	class ParticleFilterBase: public FilterBase{

	protected:
		int N; // Number of particles
		MatrixXd particles; // Store current (quaternion) particles, updated after every iteration
		VectorXd sensor_noise_cov; // Noise covariance of ALL sensors 

	public:

		// Constructor
		ParticleFilterBase(const VectorXd &IC_mean, const VectorXd &IC_std, const int num_particles)
			: FilterBase(IC_mean, IC_std), N(num_particles) {}

		// Initialize a particle filter, generate initial particles
		virtual void initialize(std::default_random_engine &generator) override;

		// Generate initial particles from a Gaussian distribution
		void initialize_gaussian(std::default_random_engine &generator);

		// No need to define update() method for ParticleFilter, use override version in BPF and FPF.

		virtual std::ostream& message(std::ostream &out) const override;

	};


} // End of namespace attitude_estimation



#endif




