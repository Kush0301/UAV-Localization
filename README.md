### Localization of a UAV during its transition between an Outdoor Environment and a GPS Denied Environment




# Aim:

1. Stablize the motion of an Unmanned Aerial Vehicle during its transition through an Outdoor Environment and a GPS denied environment.
2. Localization in an indoor environment to be achieved through the fusion of an IMU with a Monocular Camera
3. Rely on a co-variance based switching algorithm. Odometry to be provied by IMU and Camera until GPS readings within a considerable co-variance range are received.


# Simulation and Software Platform:

1. ROS Melodic
2. Gazebo 9
3. Pixhawk Flight Controller


# Progress 
1. Estimated the trajectory of the UAV using self-generated datasets from Gazebo using Monocular Visual Odometry upto a relative scale.
2. Fused this odometry information with that of an IMU through an Extended Kalman Filter.
3. Implemented teleoperation on the offboard mode of a Pixhawk.

# Currently working on

1. Integrating the fused odometry information with the offboard mode by disabling GPS information.
2. Tightly coupled fusion of an IMU with that of a Camera using Pose Graphs.
