
/* The World class handles defining and simulating all types of objects: 
   targets, sensors, and filters. The World is also an information hub 
   where an object can access information/data of another object. 
   For example, a sensor can access target state, or a filter can access sensor measurements, 
   all via the World. */

#ifndef WORLD
#define WORLD

#include <iostream>
#include <vector>
#include <random>

#include <Eigen/Dense>

// using namespace std; 
// using namespace Eigen;


namespace attitude_estimation{

	// Forward declaration
	class TargetBase; 
	class SensorBase;
	class FilterBase;

	class World
	{
		Eigen::RowVectorXd time;
		int dim_space; // Dimension of space, e.g. dim_space=3 for SO(3)
		int dim_state; // Dimension of target/filter state, e.g. dim_filter=4 if quaternion is used
		int num_times; // Total number of simulation iterations
		TargetBase *target; // Pointer to the target, only one target is considered
		std::vector<SensorBase*> sensor_list; // List of pointers pointing to the added sensors
		std::vector<FilterBase*> filter_list; // List of pointers pointing to the added filters
		

	public:

		// Constructor
		World(double start, double stop, double step, int dim_space, int dim_state);

		// The world give public access to acquire space-time dimensions and reach all the relevant objects via pointers
		int get_time() const;
		int get_space_dim() const;
		int get_state_dim() const;
		TargetBase* get_target() const;
		std::vector<SensorBase*> get_sensor() const;
		std::vector<FilterBase*> get_filter() const;

		// Overload << operator
		friend std::ostream& operator<<(std::ostream &out, World &world);

		// Add a target
		void add_target(TargetBase *t);

		// Simulate the target
		void simulate_target(std::default_random_engine &generator);

		// Add a sensor
		void add_sensor(SensorBase *s);

		// Simulate all sensors in sensor_list
		void simulate_sensors(std::default_random_engine &generator);

		// Add a filter
		void add_filter(FilterBase *f);

		// Simulate all filters in filter_list
		void simulate_filters(std::default_random_engine &generator);

		
	};

} // End of namespace attitude_estimation



#endif

