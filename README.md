# backpack_ROS
URDF files and backpack arm movement code through ROS

We use moveit to visualize the backpack in rviz and simulate its movements. To install, run "sudo apt install ros-kinetic-moveit". Once the package is installed, run "roslaunch backpack_config demo.launch" to load the urdf file of the backpack in rviz. The user should be able to see a model of a person with the backpack, and can interact with it through the various plugin tools.

To execute a trajectory for the robot arm to go to a defined pose, run "rosrun move_backpack move_to_defined_pose.py (pose id)", e.g, "rosrun move_backpack move_to_defined_pose.py pose_1". There are four poses currently defined, labeled "pose_1", "pose_2" "pose_3" and "pose_4"

To add an object to the planning scene such that the arm could choose a trajectory which avoids collision into it, run "rosrun move_backpack add_objects.py (x position) (y position) (z position) (width) (length) (height)". The object is symbolized as a solid rectangular, and the command takes in six additional commands. Multiple objects could be added to the planning scene, and they are labeled "box1" "box2" "box3", etc., based on the order they were added. 

To remove an object from the planning scene, run "rosrun move_backpack remove_objects.py (box id)", e.g, "ronrun move_backpack remove_objects.py box1" removes the first object added into the scene. 

To add a wall into the planning scene, run "ronrun move_backpack add_wall.py (height)". 
