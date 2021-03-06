#include <iostream>
#include <vector>
#include <random>

#include <Eigen/Dense>

#include "World.h"
#include "Target.h" 
#include "Sensor.h"
#include "Filter.h"

// using namespace std; 
// using namespace Eigen;


namespace attitude_estimation{

	World::World(double start, double stop, double step, int dim_space, int dim_state){
		// dim = d;
		num_times = static_cast<int>((stop-start)/step) + 1;
		Eigen::RowVectorXd Time = RowVectorXd::LinSpaced(num_times, start, stop);
		time = Time;
		this->dim_space = dim_space; // Use this-> identifier names coincide!
		this->dim_state = dim_state;
	}

	int World::get_time() const {return num_times;}

	int World::get_space_dim() const {return dim_space;}

	int World::get_state_dim() const {return dim_state;}

	TargetBase* World::get_target() const {return target;}

	std::vector<SensorBase*> World::get_sensor() const {return sensor_list;}

	std::vector<FilterBase*> World::get_filter() const {return filter_list;}

	// Overload << operator
	std::ostream& operator<<(std::ostream &out, World &world){
		out << "Time length: " << world.num_times << '\n';
		return out;
	}

	// Add a target
	void World::add_target(TargetBase *t){
		target = t;
		t->world = this; // World needs to be a friend of Target in order to do this
	}

	// Simulate target
	void World::simulate_target(std::default_random_engine &generator){
		std::cout << "Simulating target..." << '\n';
		for (int TI=0; TI<num_times-1; ++TI){
			target->move(TI, time[TI+1]-time[TI], generator);
		}
	}

	// Add a sensor
	void World::add_sensor(SensorBase *s){
		sensor_list.push_back(s);
		s->world = this;
	}

	// Simulate all sensors in sensor_list
	void World::simulate_sensors(std::default_random_engine &generator){
		std::cout << "Simulating " << sensor_list.size() << " sensors..." << '\n';
		for (int TI=0; TI<num_times; ++TI){
			double dt;
			if (TI == 0){dt = time[TI+1]-time[TI];}
			else {dt = time[TI]-time[TI-1];}
			for (SensorBase *s: sensor_list){
				s->observe(TI, dt, generator);
			}
		}
	}

	// Add a filter
	void World::add_filter(FilterBase *f){
		filter_list.push_back(f);
		f->world = this;
	}

	// Simulate all filters in filter_list
	void World::simulate_filters(std::default_random_engine &generator){
		std::cout << "Simulating " << filter_list.size() << " filters..." << '\n';
		for (int TI=0; TI<num_times-1; ++TI){
			std::cout  << "TI = " << TI << std::endl;
			for (FilterBase *f: filter_list){
				f->update(TI, time[TI+1]-time[TI], generator);
			}
		}
	}



} // End of namespace attitude_estimation



