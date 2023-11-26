The goal of this assignment is to implement the RRT algorithm for sampling-based motion planning. We will make use of the MoveIt! software framework to help with some needed auxiliary calls, but the motion planning algorithm will be implemented as part of the homework.

Motion Planning:

Your node must subscribe to a topic called "/motion_planning_goal" of the data type "geometry_msgs/Transform". Each message received on this topic represents a new goal; you must plan and execute an appropriate joint trajectory.
The start point of motion planning is a goal expressed in joint space. The goal you receive is in end-effector space so you must perform IK on it to obtain a goal configuration in joint space. However, the IK algorithm must know to only return solutions that avoid collisions (if they exist). Our starter code (the mp_starter.py script, as described below) provides you with a function that performs collision-aware IK. (Incidentally, this is done by using a MoveIt! service, but knowledge of this is not required for implementation.) As extra credit, you can instead use your own numerical IK implementation (from Assignment 3), but you must make it collision-aware first.
Once you have a goal in joint space, implement the RRT algorithm as discussed in class and presented on the handout.
You will also need a call that verifies if a specific set of joint values produces a "valid" (collision-free) configuration. Our starter code also provides a function to do that.
Note that, for the KUKA arm, each joint range is between -PI and PI. You will need to sample random joint values inside this entire range if the planner is to succeed with the more complicated obstacles.
After planning an appropriate path to the goal, you must publish it as a trajectory. You will publish the result on the topic "/joint_trajectory" of the type "trajectory_msgs/JointTrajectory". Use "rosmsg show JointTrajectory" to see what this message type looks like. The message you publish should be populated as follows:
the joint_names field should contain the names of your joints, in the same order as the joint values that make up your trajectory.
the points[] field should contain a list of waypoints in joint space. Each entry in this list is of the type "JointTrajectoryPoint". In each of these entries, you must only fill in the "positions" field with the joint values at that specific waypoint. Leave all the other fields (e.g.  velocities, accelerations, etc.) empty.
Note that, nominally, a trajectory should also contain timing information; however, to simplify the assignment, we only ask you to fill in the position component of the trajectory.
Note that your path must always start from the current robot configuration. To know what your start configuration is, subscribe to the "/joint_states" topic, as in previous assignments.
As discussed in class, you must "shortcut" the path returned by the RRT. Do that by directly connecting any two waypoints in your path that can be connected via a collision-free straight line segment in joint space. Here is an illustration of shortcutting:


After shortcutting, you must re-sample your trajectory in joint space. For each segment, add a number of internal samples equally spaced apart inside the segment. The number of internal samples for each segment should be the smallest possible such that no consecutive points on the path are farther than 0.5 to each other. If you do not re-sample your trajectory, the robot will move a little bit and then immediately stop. Note that the number of internal samples you will add could be different for each segment. Here is an illustration of re-sampling:


Set up and execution

To install MoveIt!, use the following command:
sudo apt-get install ros-kinetic-moveit
Due to an inconsistency in the assimp package included with Ubuntu 16.04, you also need to run the following fix:
sudo apt-get update
sudo apt-get install python-pip
sudo -H pip install pyassimp --upgrade
Download the Assignment 4 starter code Download Assignment 4 starter code. This tarball will contain a src directory which you can use to start your workspace. In this directory you will find a number of packages that we provide. Don't forget to catkin_make your workspace.
In the scripts directory of the motion_planning package you will find mp_starter.py - this is the starter code that provides the implementation of collision-aware Inverse Kinematics and joint state validity check. Copy this to your own package, and rename to match naming conventions (see Submission Info).
In this Assignment we only use the Kuka LWR model (UR5 is extra credit). To launch the robot, use:
roslaunch motion_planning kuka_lwr.launch
This will also start RViz, where you will see the robot. You will also see a familiar ring-and-arrows control, which you can drag to set the goal of the motion planner.
You must create your own package, following the naming conventions that apply to all Assignments. Inside this package, you must create a motion planning node called mp.py. This node must perform the function described above. In a separate terminal, run your node like so:
rosrun [your_package_name] mp.py
When you right-click on the rings-and-arrows control and select "Move Arm" your node will receive a goal to motion plan to the current position of the control. If your node does all its work correctly and publishes a valid trajectory, the arm will move to the goal.
You can add obstacles to the scene by right-clicking the rings-and-arrows control and selecting "Obstacles". You will have multiple choice of obstacles.
Your node must produce collision-free movement. Make sure to visually inspect your paths to make sure the robot does not go through the obstacle.
We will grade your submission as follows:

Correct execution of paths with no obstacles: 5 points. Robot has 10 seconds to compute and execute the path.
Correct execution of paths with simple obstacle: 5 points. Robot has 30 seconds to compute and execute the path.
Correct execution of paths with hard obstacle: 5 points. Robot has 120 seconds to compute and execute the path.
All timing is computed from the time you click "Move Arm" on the marker and until the robot has completed the path.
The super-hard obstacle is provided for your testing pleasure and not included in grading.
Extra Credit 1: 3 points. Implement your own numerical IK function, and use it instead of the MoveIt version. Note that your IK function must also be collision-aware and only return valid solutions.  NEW 11/14: You may use the provided "is_state_valid" function provided in the starter code to make your numerical IK collision aware.
Extra Credit 2: 4 points. Provide a solution that also works on the UR5 robot. You might need to provide additional packages to make this happen - figure it out! The online MoveIt documentation is a good place to start. The same mp.py script must work on both robots; however, you will likely need to provide a launch file for the UR5 to be used instead of kuka_lwr.launch.
