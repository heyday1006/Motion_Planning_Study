Assignment 3 - Cartesian Control and Numerical IK

The goal of this assignment is to write a node capable of performing Cartesian Control and Numerical IK. Here are the formal requirements for the node:

Cartesian Control:

Your node must subscribe to a topic called "/cartesian_command" with the data type "cartesian_control/CartesianCommand". Each message received on this topic represents a command for the cartesian controller as follows:
the field "x_target" represents the desired transform of the end-effector w.r.t. the robot base
if the field "secondary_objective" is True, then you should also attempt to execute the secondary objective. In this case, the field "q0_target" holds a target value that the first joint of the robot must achieve as a secondary objective. If "secondary_objective" is False, then you can ignore this value.
Your node must publish to a topic called "/joint_velocities" of the type JointState. You must publish on this topic the joint velocities to be executed by the robot. You must fill in two fields of the message: the "string[] name" field must contain a list of joint names, and the "float64[] velocity" must contain the commanded joint velocities, with the joints appearing in the same order in both fields.
Whenever you receive a command on "/cartesian_command", you must compute appropriate joint velocities in response, then publish them on the "/joint_velocities" topic. You only need to publish velocities once for every time you receive a command.
Numerical IK:

Your node must subscribe to a topic called "/ik_command" with the data type Transform. Each message received on this topic represents a desired end-effector pose relative to the root of your robot.
Your node must publish to a topic called "/joint_command" of the type JointState. You must publish on this topic the joint values that place the end-effector at the pose commanded on the topic above. You must fill in two fields of the message: the "string[] name" field must contain a list of joint names, and the "float64[] position" must contain the commanded joint values, with the joints appearing in the same order in both fields.
Whenever you receive a command on "/ik_command", you must compute appropriate joint positions in response, then publish them on the "/joint_command" topic. You only need to publish the resulting positions once for every time you receive a command.
Shared information:

Your node can read URDF information from the parameter server as in the previous assignment.
Your node can subscribe to the "/joint_states" topic to obtain current joint values as in the previous assignment.
Set up and execution:

Download the Assignment 3 starter code Download Assignment 3 starter code. This tarball will contain a src directory which you can use to start your workspace. In this directory you will find a number of packages that we provide. Don't forget to catkin_make your workspace.
We provide two robot URDF's - a Kuka LWR and a Universal UR5. Your code must function properly on both. To start each of the two robots, use one of the following commands:
roslaunch cartesian_control kuka_lwr.launch
roslaunch cartesian_control ur5.launch
This will start a simulated robot, load the URDF to the parameter server, and start RVIZ with a proper configuration file. You will also see two Interactive Markers on your robot: a rings-and-arrows control around the end-effector, and a ring around the first link.
You must create your own package, following the naming conventions that apply to all Assignments. Inside this package, you must create a Cartesian Control / Numerical IK node called ccik.py. This node must perform both functions described above. In a separate terminal, run your node like so:
rosrun [your_package_name] ccik.py
Once your node is running and doing its job correctly, you can test the behavior as follows:

The Interactive Marker on the end-effector has two modes: Cartesian Control Mode and Inverse Kinematics Mode. You can select which one is active by right-clicking anywhere on the marker (on the rings or on the arrows). At start-up, the Cartesian Control Mode is active by default.
Cartesian Control Mode: while you are moving the control around, it will continuously issue Cartesian Control commands to your node. If you release the control, it will move back to the current pose of the end-effector. Dragging the ring control around the first link will allow you to issue secondary objective commands.
Inverse Kinematics Mode: you can position the control anywhere you want, without any commands being issued. Once you are happy with the position of the control, right-click the control again and select "IK to Here". This will issue an IK command to your node with the current pose of the control as a target.
Assumptions and tips:

Remember to check the implementation details handoutActions  for a step-by-step walk-through of your algorithms.
You can assume that all the joints of your robot are revolute joints. Furthermore, all joint axes are either x, -x, y, -y, z, or -z. In other words, the local joint axis must be one of: (1,0,0), (-1,0,0), (0,1,0), (0,-1,0), (0,0,1), (0,0,-1).
To convert a rotation from the matrix representation to the angle-axis representation, you can use the function provided in this script Download this script.
Remember that the coordinate frame of the end-effector (which is the one you are controlling) is the coordinate frame of the last link of the robot (not necessarily the coordinate frame of the last joint).
You should implement most of the core algorithm as a function that is called both when you need to perform Cartesian Control and when you need to perform Inverse Kinematics.
As part of the algorithm, you will need transforms from the root coordinate frame into the coordinate frame of every joint of your robot. You can re-use your Forward Kinematics code from the previous assignment. Be careful: the coordinate frame of a joint is *before* taking into account the rotation introduced by the joint itself (the rotation by the joint value around the joint axis).
Note that you do not need to be publishing any of these transforms; this time, a separate ROS component is also performing Forward Kinematics and publishing the result to TF (this is why the robot shows up correctly in RViz even without you doing anything). In theory, you could skip the FK part and simply get the relevant transform information directly from TF (note: we have not tried this practice).
(New 10/24) Code organization tip: you will likely have (at least) three ROS callbacks in your code: one for listening to the /joint_states topic, and one each for listening to the two commands you are implementing (Cartesian Control and IK). Do not try to perform Forward Kinematics inside your /joint_states callback. Instead, here you should simply store the new joint angles that you have received. Have a separate Forward Kinematics function, that takes as arguments the joint angles that it needs to operate on; this function should be called as needed from the Cartesian Control and IK callbacks. Here is an example code structure Download example code structure I used in my own implementation. You are allowed to use it exactly as it is, to modify it as you see fit, or simply to use it as inspiration for your own code.
(New 10/24) There is a bug in the GUI: if you launch everything from scratch and, without moving the rings-and-arrow control at all, you switch to IK mode and immediately ask for "IK to here" then the command that gets sent to your node does not mirror the actual position of the control. To avoid this, after launching, simply drag the rings-and-arrow control a little bit in any direction before requesting any IK results.