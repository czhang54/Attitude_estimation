
add_library(attitude_estimation_source
	Lie_group.cpp 
	World.cpp 
	Target.cpp 
	Sensor.cpp 
	Filter.cpp 
	MultiplicativeEKF.cpp 
	InvariantEKF.cpp 
	ParticleFilter.cpp 
	FeedbackParticleFilter.cpp
)

# Include headers from a separate directory
include_directories(../include) 

