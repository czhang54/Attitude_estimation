
/* This header defines all types of the target objects */

#ifndef TARGET
#define TARGET

#include <iostream>
#include <vector>

#include <Eigen/Dense>

// using namespace std;
// using namespace Eigen;


namespace attitude_estimation{

	using namespace Eigen;

	class World; // Forward declaration

	/* ########## Base class for all targets ########## */
	class TargetBase
	{
	protected:

		World *world; // A pointer to the World, set when the target is added to World
		MatrixXd state; // (Quaternion) state at all time steps, accessed by sensors
		MatrixXd angular_velocity; // Angular velocity vector at all time steps, accessed by filters
		VectorXd IC; // Initial attitude of target
		VectorXd IC_std; // Standard deviation of initial distribution, not used if target initialization is deterministic
		VectorXd process_noise_std; // Noise parameter of process noise added to the target motion

	public:

		// Constructor
		TargetBase(const VectorXd &IC, const VectorXd &IC_std, const VectorXd &process_noise_std)
			: IC(IC), IC_std(IC_std), process_noise_std(process_noise_std) {}
	 
	 	// Give access to the following information of target, used by sensors and filters
		MatrixXd& get_state() {return state;}
		MatrixXd& get_velocity() {return angular_velocity;}
		VectorXd& get_process_noise() {return process_noise_std;}

		// Initialize a target
		void initialize(); // Set time horizon and initial condition

		// Simulate a target according to a model
		virtual void move(int TI, double dt, std::default_random_engine &generator);

		friend std::ostream& operator<<(std::ostream &out, const TargetBase *t){
			return t->message(out);
		}

		virtual std::ostream& message(std::ostream &out) const {
			out << "Tracking a target" << '\n';
			return out;
		}

		friend World; // MUST friend the entire World class!
		// friend World::add_target(Target *t);

	};


	/* A target whose angular velocity model is sinusoidal */
	class Sinusoidal: public TargetBase
	{

	public:

		// Constructor
		Sinusoidal(const VectorXd &IC, const VectorXd &IC_std, const VectorXd &process_noise_std)
			: TargetBase(IC, IC_std, process_noise_std) {}

		// Simulate (move) the target for one iteration
		virtual void move(int TI, double dt, std::default_random_engine &generator) override;

		virtual std::ostream& message(std::ostream &out) const override {
			out << "Tracking a target with sinusoidal angular velocity..." << '\n';
			return out;
		}

	};


} // End of namespace attitude_estimation
















#endif