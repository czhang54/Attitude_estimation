#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <map>
#include <random>

#include <Eigen/Dense>

// Header guard is needed to avoid duplicate include
#include "../include/World.h"
#include "../include/Target.h"
#include "../include/Sensor.h"
#include "../include/Filter.h"
#include "../include/Lie_group.h" // To use quaternion data type defined therein

using namespace std;
using namespace Eigen;


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
	default_random_engine generator;
	generator.seed(static_cast<unsigned int>(std::time(0)));


	/* Simulation parameters */

	// Process noise is defined in tangent space of the manifold, hence its dimension equals dim_space
	VectorXd process_noise_std = VectorXd::Constant(dim_space, 0.2);
	VectorXd sensor_noise_std = VectorXd::Constant(dim_sensor, 30*M_PI/180.0);

	// Target and filter initialization
	quaternion IC_target = angle_axis(Vector3d(3,1,4), 180); // Initial attitude of target
	quaternion IC_filter(1,0,0,0); // Mean of initial filter estimate
	double IC_std_degree = 60; // Standard deviation (in degree) of initial filter estimate
	VectorXd IC_std = VectorXd::Constant(3, IC_std_degree*M_PI/180.0); 


	/* Simulate target */
	Sinusoidal sinusoidal(IC_target, IC_std, process_noise_std);
	Target *target = &sinusoidal;

	world.add_target(target);
	target->initialize(); // Initialization is done after being added to world
	cout << target;
	world.simulate_target(generator); // Simulate target and obtain target trajectory


	/* Generate sensor measurements */
	// Accelerometer: measures gravity vector in sensor frame
	const Vector3d gravity(0,0,1); // Gravity vector
	Accelerometer accelerometer(dim_sensor, sensor_noise_std, gravity);
	world.add_sensor(&accelerometer);
	accelerometer.initialize();

	// Magnetometer: ma=easures local megnetic field vector in sensor frame
	const Vector3d magnetic_field(1,0,1); // Magnetic field vector
	Magnetometer magnetometer(dim_sensor, sensor_noise_std, magnetic_field);
	world.add_sensor(&magnetometer);
	magnetometer.initialize();

	for (Sensor *s: world.get_sensor()){cout << s;}

	world.simulate_sensor(generator); // Simulate sensors and obtain measurements

	/* Build MEKF */
	MEKF *mekf = new MEKF(IC_filter, IC_std);
	world.add_filter(mekf);
	mekf->initialize(generator);
	cout << mekf;

	/* Build IEKF */
	IEKF iekf(IC_filter, IC_std);
	world.add_filter(&iekf);
	iekf.initialize(generator);
	cout << &iekf;

	/* Build particle filter */
	ParticleFilter pf(IC_filter, IC_std, num_particles, 0.1);
	world.add_filter(&pf);
	pf.initialize(generator);
	cout << &pf;

	/* Build FPF */
	FPF *fpf = new FPF(IC_filter, IC_std, num_particles, "galerkin", 10, 50);
	world.add_filter(fpf);
	fpf->initialize(generator);
	cout << fpf;

	world.simulate_filter(generator); // Simulate all the filters


	cout << "Simulation of attitude estimation finished!" << '\n';

	return 0;

}



