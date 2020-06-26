# robocol_ur3
Robocol packages for UR3 arm.

To run Gazebo sim:

roslaunch ur_gazebo ur3.launch

To run Planning Execution:

roslaunch ur3_moveit_config ur3_moveit_planning_execution.launch sim:=true

To run Rviz sim:

roslaunch ur3_moveit_config moveit_rviz.launch config:=true
