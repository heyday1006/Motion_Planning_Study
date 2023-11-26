The goal of this assignment is to implement an Extended Kalman Filter. You are estimating the state of a mobile robot; the state consists of the robot's 2D position and orientation (x,y,theta). The command to the robot consists of a forward translational velocity vel_trans and a rotational velocity vel_ang. The dynamic model of the robot thus is as follows:

x(k+1) = x(k) + t * vel_trans * cos(theta)
y(k+1) = y(k) + t * vel_trans * sin(theta)
theta(k+1) = theta(k) + t * vel_ang
where t is the length of the time step, which you can assume to be 0.01 seconds. Every time step, the robot will publish both vel_trans and vel_ang, but, to account for errors in the system, you should assume white noise with covariance 0.1 applied to each variable. 

In addition to publishing the commanded velocities, the robot can also localize itself with respect to a number of landmarks in the scene. Every time a landmark is within range of the robot's sensors, the robot will record its distance to the landmark, as well as the bearing of the landmark with respect to the robot's orientation. Assuming the robot is at coordinates (xr, yr, thetar) and the landmark is at coordinates (xl, yl), the range and bearing are computed as follows:

range = math.sqrt( (xr-xl)*(xr-xl) + (yr-yl)*(yr-yl) )
bearing = math.atan2(yl-yr, xl-xr) - thetar
The robot will publish a range and bearing for each of the landmarks that it can see at a given time.

You must write a node called "estimator.py" that implements an EKF and uses the information above to compute and publish an estimate of the robot's position. Topics are as follows:

"/sensor_data": on this topic you will receive messages of the type "state_estimator/SensorData" which contain all the information described above. Every time you receive a new message on this topic, you must advance your estimation by one time step using the information received. You only need to advance by a time step when you receive new information on this topic. Futhermore, you can always assume that the length of a time step is 0.01 s, regardless of how long it actually takes between consecutive messages to arrive.
"/robot_pose_estimate": on this topic, you must publish messages of the type "geometry_msgs/Pose2D" containing the current estimate of the robot's pose. You must publish a message every time you advance your estimation by a time step (i.e. whenever receiving a sensor data message on the topic above).
You can assume that the robot starts at coordinates (0,0,0).
While the robot is also publishing it's real position at any given time (so it can be plotted by the GUI, described below), your estimator is obviously NOT allowed to subscribe to that topic (and we will change the name of that topic to something unknown for grading).
Set-up and execution:

Download the Assignment 5 starter code Download Assignment 5 starter code. It will contain a single package that we provide called "state_estimator".
You can start the robot using
rosrun state_estimator robot.py
The robot will start moving in random directions as soon as you start it. Note that you should start your estimator node before starting the robot, so that the two are synchronized: if you start the robot first, then the robot will move around by the time the estimator starts, so the start position of (0,0,0) assumed by your estimator will no longer be valid. You can reset the robot by stopping and re-starting it.
A simple GUI is available to show you the location of the real robot (red), your estimate (blue), the landmarks (blue stars), and the landmarks with range of the robot (red stars). You can start the GUI as follows:
rosrun state_estimator gui.py
You should start the GUI first (before the estimator and the robot) and let it run - it will wait for information to be published. If the GUI is too small on your screen, feel free to edit state_estimator/scripts/gui.py to get it to look right for you.
Expected behavior:

When no landmark is within range, the estimate and the robot can deviate from each other. This can also happen when a single landmark is in range (why?). 
The estimate can also occasionally "jump around".
However, when two landmarks are visible simultaneously, or when a new landmark is acquired, the estimate shold quickly converge to the true robot location, to the point where only one of them is visible.
I will show the behavior of my implementation in class on Thu 12/7 as a reference point.
Tips and additional information

(12/11) Computation of innovation in EKF: the State Estimation handout mentions that the prediction and update steps are similar to the KF, after computing the F and H Jacobians. However, Eq. (48) is not applied ad literam: there is no need to linearize computation of the "expected" sensor data, since we do have an explicit sensor model in the h(x) function. Thus, the innovation can be computed as nu = y(k+1) - h ( x^ (k+1) ), where x^ stands for "x hat", or the predicted state.