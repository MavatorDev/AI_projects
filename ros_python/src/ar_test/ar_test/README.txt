# Advanced Robotic Systems - EMS628U/728P
# Coursework: Cubic Trajectory Generator ROS2 Package

Code developed by: MANUELA VALENCIA TORO, QMUL
Student ID: 240429850

## Package Description
There are two packages to install in your workspace: ar_test and ar_interface to generate and plot cubic trajectories between randomly generated points. 
## To run the package:

1. Unzip the folder inside the src folder of your ROS2 workspace.
2. Build the workspace with 'colcon build'
3. Source the workspace 'source ~/ros2cw_ws/install/setup.bash'
4. Execute the launcher 'ros2 launch ar_test cubic_traj_gen.launch.py'
5. Open a new terminal and Execute rqt to open the graph and add the topics you want to see
