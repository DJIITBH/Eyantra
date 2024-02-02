# Cosmo Logistic
I implemented an autonomous warehous setup in Gazebo as a participant of Eyantra Robotics Competition.

# UR5
Manipulation of UR5 robotic arm was based on MoveIt packages.

# Mobile Robot
A mobile robot having Lidar, IMU, Ultrasonic sensors navigated to the rack consisting of packages using Nav2 and an indoor map of the warehouse. The navigation used SLAM.
The bot delivered rack in front of the arm such that arm could pick and place the packages.

# Pose Estimation
The pose estimation of the packages was based on Aruco Markers, I extracted the depth and quaternion using Computer Vision.
