# rbx1_motion_planning package
## Usage
The package contains 1 node which itself contains 3 action servers, each one controls the arm given different goals.

To start the node run the following from terminal: <br>
`rosrun rbx1_motion_planning rbx1MotionPlanningServer.py`

Or to start the node and the Rvis simulator run the following from terminal: <br>
`roslaunch rbx1_scripts rbx1_init.launch`

## Action Servers
The package contiains 3 action servers within the rbx1MotionPlanningServer node:

- executeJointGoal_as (Joint Goal)
- executePoseGoal_as (Pose Goal)
- executePositionGoal_as (Position Goal)

### Joint Goal
The executeJointGoal_as should be provided with a joint goal as an array in the form of a **std_msgs/Float64MultiArray** ROS message with 6 elements where each one is the angle of a joint in **radians**, and will output a boolean result of true or false depending on wether the movement of the arm was successful. The correct format of the action goal can be initialised using the function `goal = executeJointGoalGoal()`, and then to input the goal data do: `goal.target.data = [j1, j2, j3, j4, j5, j6]`

### Pose Goal
The executePoseGoal_as should be proveded with a pose goal as a pose in the form of a **geometry_msgs/Pose** ROS message with a position and a quaternion orientation, and will output a boolean result of true or false depending on wether the movement of the arm was successful. The correct format of the action goal can be initialised using the function `goal = executePoseGoalGoal()`, and then to input the goal data do: `goal.target.position.x =` to set the x coordinate (and similar for y and z) and `goal.target.orientation.x =` to set each element of the quaternion (and similar for y z and w).

### Position Goal
The executePostionGoal_as should be provede with a position goal as an array in the form of a **std_msgs/Float64MultiArray** ROS message with 3 elements where each one is the x y and z cartesian coordinate respectively, and will output a boolean result of true or false depending on wether the movement of the arm was successful. The correct format of the action goal can be initialised using the function `goal = executePositionGoalGoal()`, and then to input the goal data do: `goal.target.data = [x,y,z]`