# The forward translational velocity commanded to the robot
float64 vel_trans

# The rotational velocity commanded to the robot
float64 vel_ang

# The readings of landmarks with the range of the robot's sensors
# Can be empty, if no landmarks are nearby
state_estimator/LandmarkReading[] readings
