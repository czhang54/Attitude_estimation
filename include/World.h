#ifndef WORLD
#define WORLD

#include <iostream>
#include <string>
#include <vector>
#include <array>
#include <cmath>
#include <random>

#include <Eigen/Dense>

// using namespace std; 
using namespace Eigen;


namespace Attitude_estimation{

	class Target; 
	class Sensor;
	class FilterBase;

	class World
	{
		RowVectorXd time;
		int dim_space; // Dimension of space, e.g. dim_space=3 for SO(3)
		int dim_state; // Dimension of target/filter state, e.g. dim_filter=4 if quaternion is used
		int num_times; // Length of simulation iterations
		Target *target; // Pointer to the target, only one target is considered
		std::vector<Sensor*> sensor_list; // List of pointers pointing to the added sensors
		std::vector<FilterBase*> filter_list; // List of pointers pointing to the added filters
		

	public:
		World(double start, double stop, double step, int dim_space, int dim_state);

		// The world give public access to acquire space-time dimensions and reach all the relevant objects via pointers
		int get_time() const;
		int get_space_dim() const;
		int get_state_dim() const;
		Target* get_target() const;
		std::vector<Sensor*> get_sensor() const;
		std::vector<FilterBase*> get_filter() const;

		// Overload << operator
		friend std::ostream& operator<<(std::ostream &out, World &world);

		// Add a target
		void add_target(Target *t);

		// Simulate the target
		void simulate_target(std::default_random_engine &generator);

		// Add a sensor
		void add_sensor(Sensor *s);

		// Simulate all sensors in list
		void simulate_sensor(std::default_random_engine &generator);

		// Add a filter
		void add_filter(FilterBase *f);

		// Simulate all filters in list
		void simulate_filter(std::default_random_engine &generator);

		
	};

}



#endif

