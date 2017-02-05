#ifndef WORLD
#define WORLD

#include <iostream>
#include <string>
#include <vector>
#include <array>
#include <cmath>
#include <random>

#include <Eigen/Dense>

using namespace std; 
using namespace Eigen;


class Target; // Need to declare Target class to initialize World object
class Sensor;
class Filter;


class World
{
	RowVectorXd time;
	int dim_space;
	int dim_state;
	int num_times;
	Target *target; // Need declaration of Target class as above
	vector<Sensor*> sensor_list;
	vector<Filter*> filter_list;
	

public:
	// Need start, stop, step, dim to start
	World(double start, double stop, double step, int dim_space, int dim_state);

	int get_time() const;
	int get_space_dim() const;
	int get_state_dim() const;
	Target* get_target() const;
	vector<Sensor*> get_sensor() const;
	vector<Filter*> get_filter() const;

	// Overload << operator
	friend ostream& operator<<(ostream &out, World &world);

	// Add and simulate targets
	void add_target(Target *t);

	void simulate_target(default_random_engine &generator);

	// Add and simulate sensors
	void add_sensor(Sensor *s);

	void simulate_sensor(default_random_engine &generator);

	// Add and simulation filters
	void add_filter(Filter *f);

	void simulate_filter(default_random_engine &generator);

	
};





#endif

