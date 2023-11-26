Description
In this project we consider a ROS ecosystem, which consists of a robot with a camera mounted on it as well as an object. To describe the poses of all these items, we define the following coordinate frames:

A base coordinate frame called 'base_frame'
A robot coordinate frame  called 'robot_frame'
A camera coordinate frame called 'camera_frame'
An object coordinate frame 'object_frame'
The following relationships are true:

The transform from the 'base_frame' coordinate frame to the 'object_frame' coordinate frame consists of a rotation expressed as (roll, pitch, yaw) of (0.79, 0.0, 0.79) followed by a translation of 1.0m along the resulting y-axis and 1.0m along the resulting z-axis. 
The transform from the 'base_frame' coordinate frame to the 'robot_frame' coordinate frame consists of a rotation around the z-axis by 1.5 radians followed by a translation along the resulting y-axis of -1.0m. 
The transform from the 'robot_frame' coordinate frame to the 'camera_frame' coordinate frame must be defined as follows:
The translation component of this transform is (0.0, 0.1, 0.1)
The rotation component of this transform must be set such that the camera is pointing directly at the object. In other words, the x-axis of the 'camera_frame' coordinate frame must be pointing directly at the origin of the 'object_frame' coordinate frame. 
You must create a ROS package called 'asgn1'. Inside this package, you must create a node called 'solution.py' that publishes the following transforms to TF:

The transform from the 'base_frame' coordinate frame to the 'object_frame' coordinate frame 
The transform from the 'base_frame' coordinate frame to the 'robot_frame' coordinate frame 
The transform from the 'robot_frame' coordinate frame to the 'camera_frame' coordinate frame
Additional Information
You will probably want to make use of the transformations.py library. The documentation for using that is in the library itself; you can reference the version used with ROS online on Github Links to an external site. (be careful - other versions of this file exist on the Internet, so if you just Google for it you might get the wrong one).

For a rotation expressed as roll-pitch-yaw, you can use the quaternion_from_euler() or euler_matrix() functions with the default axes convention - i.e. quaternion_from_euler(roll_value, pitch_value, yaw_value). You can also use the code in tf_examples.py for guidance.

Be careful about the order of operations. If a transform specifies that the rotation must happen first, followed by the translation (e.g. at points 1. and 2. above), make sure to follow that.

The transforms must be published in a continuous loop at a rate of 10Hz or more. 

Additional code and visual feedback

We are providing additional code that will give you visual feedback on your work. You can download the package called 'marker_publisher' and place it in your catkin workspace. Then, you must catkin_make your workspace (as the package marker_publisher contains C++ code that needs to be compiled). Then, you can launch the roslaunch file called 'markers.launch' from this package, like this:

roslaunch marker_publisher markers.launch

(Remember that you must source devel/setup.bash in your workspace before the roslaunch command above will work).

This command will bring up rviz. It will also start a node that publisher some visual markers, in the coordinate frames that you are publishing as part of the homework (feel free to look at the code in marker_publisher.cpp to get a sense of what is going on). If your solution is publishing the correct transform, these markers will show up in rviz as they are expected to. The cylinder denotes the object, the cube and arrow the robot and camera respectively. If your code works correctly, you should see the arrow point out of the cube directly at the cylinder. 