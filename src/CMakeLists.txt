
add_library(attitude_estimation_source
	Lie_group.cpp 
	World.cpp 
	Target.cpp 
	Sensor.cpp 
	Filters/Filter.cpp 
	Filters/MultiplicativeEKF.cpp 
	Filters/InvariantEKF.cpp 
	Filters/ParticleFilter.cpp 
	Filters/FeedbackParticleFilter.cpp
)

# Include headers from a separate directory
include_directories(../include) 

