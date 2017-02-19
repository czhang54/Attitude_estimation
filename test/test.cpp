
/* This test program performs a complete simulation of several filters for attitude estimation. 
   This can be a prototype for research purposes to study and compare attitude filters.

   Filters include:
   (1) Multiplicative EKF (MEKF), 
   (2) Invariant EKF (IEKF),
   (3) Particle filter (PF),
   (4) Feedback particle filter (FPF) (our new filter, see arXiv preprint: https://arxiv.org/abs/1701.02416).

   To implement the filter, target and sensor measurements are also generated. 
   Sensors include:
   (1) Accelerometer
   (2) Magnetometer

   The entire code structure is expandable, which allows future addition of more types of targets, sensors and filters.
*/

#include <iostream>
#include <vector>
#include <random>

// Eigen is required
#include <Eigen/Dense>

// Header guard is needed to avoid duplicate include

#include "World.h" // Simulation coordinator
#include "Target.h" // Contains all target objects
#include "Sensor.h" // Contains all sensor objects
#include "Lie_group.h" // Contains all computations related to Lie groups and quaternions

// Include different filters
#include "Filter.h"
#include "MultiplicativeEKF.h"
#include "InvariantEKF.h"
#include "ParticleFilter.h"
#include "FeedbackParticleFilter.h"

// Avoid/Minimize using namespace in the global scope
// using namespace std;
// using namespace Eigen;
using namespace attitude_estimation;


int main()
{
	/* Space-time parameters */
	const double start = 0.0; // Start time of simulation
	const double stop  = 0.20; // Stop time of simulation
	const double step  = 0.01; // Time step size
	const int dim_space = 3; // Dimension of the state space, e.g. dim_space = 3 for SO(3)
	const int dim_state = 4; // Nominal state dimension of target and filters, e.g. dim_filter = 4 if use quaternion
	const int dim_sensor = 3; // dim=3 for gyroscope, accelerometer, magnetometer
	const int num_particles = 100; // Number of particles

	// Define the world where all the simulated objects live
	World world(start, stop, step, dim_space, dim_state);

	// Initialize a random number generator, later passed into each object for noise generation
	std::default_random_engine generator;
	generator.seed(static_cast<unsigned int>(std::time(0)));


	/* Simulation parameters */

	// Process noise is defined in tangent space of the manifold, hence its dimension equals dim_space
	Eigen::VectorXd process_noise_std = Eigen::VectorXd::Constant(dim_space, 0.2);
	Eigen::VectorXd sensor_noise_std  = Eigen::VectorXd::Constant(dim_sensor, 30*M_PI/180.0);

	// Target and filter initialization
	quaternion IC_target = angle_axis(Eigen::Vector3d(3,1,4), 180); // Initial attitude of target
	quaternion IC_filter(1,0,0,0); // Mean of initial filter estimate
	double IC_std_degree = 60; // Standard deviation (in degree) of initial filter estimate
	Eigen::VectorXd IC_std = Eigen::VectorXd::Constant(3, IC_std_degree*M_PI/180.0); 


	/* Simulate target */
	Sinusoidal sinusoidal_target(IC_target, IC_std, process_noise_std);
	TargetBase *target = &sinusoidal_target;

	world.add_target(target);
	target->initialize(); // Initialization is done after being added to world
	std::cout << target;
	world.simulate_target(generator); // Simulate target and obtain target trajectory


	/* Generate sensor measurements */
	// Accelerometer: measures gravity vector in sensor frame
	const Eigen::Vector3d gravity(0,0,1); // Gravity vector
	Accelerometer accelerometer(dim_sensor, sensor_noise_std, gravity);
	world.add_sensor(&accelerometer);
	accelerometer.initialize();

	// Magnetometer: measures local megnetic field vector in sensor frame
	const Eigen::Vector3d magnetic_field(1,0,1); // Magnetic field vector
	Magnetometer magnetometer(dim_sensor, sensor_noise_std, magnetic_field);
	world.add_sensor(&magnetometer);
	magnetometer.initialize();

	for (SensorBase *s: world.get_sensor()){std::cout << s;}

	world.simulate_sensors(generator); // Simulate sensors and obtain measurements

	/* Build Multiplicative EKF (MEKF) */
	MultiplicativeEKF *mekf = new MultiplicativeEKF(IC_filter, IC_std);
	world.add_filter(mekf);
	mekf->initialize(generator);
	std::cout << mekf;

	/* Build Invariant EKF (IEKF) */
	InvariantEKF iekf(IC_filter, IC_std);
	world.add_filter(&iekf);
	iekf.initialize(generator);
	std::cout << &iekf;

	/* Build Particle Filter (PF) */
	ParticleFilter pf(IC_filter, IC_std, num_particles, 0.1);
	world.add_filter(&pf);
	pf.initialize(generator);
	std::cout << &pf;

	/* Build Feedback Particle Filter (FPF) */
	FeedbackParticleFilter *fpf = new FeedbackParticleFilter(IC_filter, IC_std, num_particles, "galerkin", 10, 50);
	world.add_filter(fpf);
	fpf->initialize(generator);
	std::cout << fpf;

	// Simulate all the filters
	world.simulate_filters(generator); 


	std::cout << "Simulation of attitude estimation finished!" << '\n'; 

	// Deleting the filter pointers are actually not necessary here
	delete mekf;
	mekf = NULL;
	delete fpf;
	fpf = NULL;

	return 0;

}



