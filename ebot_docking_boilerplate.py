#!/usr/bin/env python3

## Overview

# ###
# This ROS2 script is designed to control a robot's docking behavior with a rack. 
# It utilizes odometry data, ultrasonic sensor readings, and provides docking control through a custom service. 
# The script handles both linear and angular motion to achieve docking alignment and execution.
# ###

# Import necessary ROS2 packages and message types
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import euler_from_quaternion
from ebot_docking.srv import DockSw  # Import custom service message
import math, statistics
import  tf2_ros
import time 

# Define a class for your ROS2 node
class MyRobotDockingController(Node):

    def __init__(self):
        # Initialize the ROS2 node with a unique name
        super().__init__('my_robot_docking_controller')
        # self.linear_dock = False
        # self.orientation_dock = False
        # self.distance = 0.00
        # self.orientation = 0.76
        # self.rack_no = '1'
        

        # Create a callback group for managing callbacks
        self.callback_group = ReentrantCallbackGroup()
        self.robot_pose=[0.0,0.0,0.0]

        # Subscribe to odometry data for robot pose information
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)

        # Subscribe to ultrasonic sensor data for distance measurements
        self.ultrasonic_rl_sub = self.create_subscription(Range, '/ultrasonic_rl/scan', self.ultrasonic_rl_callback, 10)
        # Add another one here
        self.ultrasonic_rr_sub = self.create_subscription(Range, '/ultrasonic_rr/scan', self.ultrasonic_rr_callback, 10)

        # Create a ROS2 service for controlling docking behavior, can add another custom service message
        self.dock_control_srv = self.create_service(DockSw, 'dock_control', self.dock_control_callback, callback_group=self.callback_group)

        # Create a publisher for sending velocity commands to the robot
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)

        self.alpha_rot = 0.7
        self.alpha_trans = 0.1
        self.alpha_trans_1 = 0.5

        # Initialize all  flags and parameters here
        self.is_docking = False
        self.dock_aligned_angle = False
        self.dock_aligned_translate = False
        self.start = False
        self.home_aligned = False
        
        #         
        # 
        # 
        # 
        # 
        # 

        # Initialize a timer for the main control loop
        self.controller_timer = self.create_timer(0.1, self.controller_loop)

    # Callback function for odometry data
    def odometry_callback(self, msg):
        # Extract and update robot pose information from odometry message
        # msg=Odometry()
        self.robot_pose[0] = msg.pose.pose.position.x
        self.robot_pose[1] = msg.pose.pose.position.y
        quaternion_array = msg.pose.pose.orientation
        orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
        roll,pitch, yaw = euler_from_quaternion(orientation_list)
        self.robot_pose[2] = yaw

    # Callback function for the left ultrasonic sensor
    def ultrasonic_rl_callback(self, msg):
        # print('hi left')
        self.usrleft_value = msg.range

    # Callback function for the right ultrasonic sensor
    #
    #
    def ultrasonic_rr_callback(self, msg):
        # print('hi')
        self.usright_value = msg.range

    # Utility function to normalize angles within the range of -π to π (OPTIONAL)
    def normalize_angle(self, angle):
        
        pass

    # Main control loop for managing docking behavior

    def controller_loop(self):

        # The controller loop manages the robot's linear and angular motion 
        # control to achieve docking alignment and execution
       
        # if self.is_docking:
        #     print("hi")
        #     # ...
            # Implement control logic here for linear and angular motion
            # For example P-controller is enough, what is P-controller go check it out !
            # ...
            if self.dock_aligned_angle is False and self.start:
                # if self.linear_dock:
                    orient_change = self.robot_pose[2] - self.orientation
                    twist = Twist()
                    twist.linear.x ,twist.linear.y, twist.linear.z = 0.0,0.0,0.0
                    twist.angular.x , twist.angular.y ,twist.angular.z = 0.0, 0.0, -self.alpha_rot * orient_change
                    print(self.alpha_rot * orient_change)
                    if abs(self.alpha_rot*orient_change) < 0.01:
                        self.dock_aligned_angle = True
                        twist.linear.x ,twist.linear.y, twist.linear.z = 0.0,0.0,0.0
                        twist.angular.x , twist.angular.y ,twist.angular.z = 0.0, 0.0, 0.0
                        self.publisher_.publish(twist)
                    

                    self.publisher_.publish(twist)
                
            if self.dock_aligned_angle and self.start and not self.dock_aligned_translate:
                if self.linear_dock:
                    linear_speed = -self.alpha_trans * self.usright_value
                    print(linear_speed)
                    twist = Twist()
                    twist.linear.x ,twist.linear.y, twist.linear.z = linear_speed,0.0,0.0
                    twist.angular.x , twist.angular.y ,twist.angular.z = 0.0, 0.0, 0.0
                    self.publisher_.publish(twist)
                    print(linear_speed)
                    if abs(linear_speed) < 0.01 :
                        print('rack mei hi hai')
                        
                        twist.linear.x ,twist.linear.y, twist.linear.z = 0.0,0.0,0.0
                        twist.angular.x , twist.angular.y ,twist.angular.z = 0.0, 0.0, 0.0
                        if self.distance>0:
                                # print('aage jaega')
                            #   linear_error = abs(self.distance - self.robot_pose[0])
                            #   print(linear_error)
                            #   linear_home_vel = -self.alpha_trans * linear_error
                                start = time.time()
                                j= self.robot_pose[0]
                                print(j," initial")
                                # print(self.distance,"distance")
                                if((self.distance - self.robot_pose[0])>0):
                                    self.error = self.distance - self.robot_pose[0]
                                    print(self.error*self.alpha_trans_1," error")
                                    twist.linear.x ,twist.linear.y, twist.linear.z = -self.error*self.alpha_trans_1,0.0,0.0
                                    twist.angular.x , twist.angular.y ,twist.angular.z = 0.0, 0.0, 0.0
                                    # print("publish")s
                                    self.home_dock =  True
                                    self.publisher_.publish(twist)
                                if (self.alpha_trans_1*self.error<0.02):
                                    self.dock_aligned_translate = True
                                    twist.angular.x , twist.angular.y ,twist.angular.z = 0.0, 0.0, 0.0
                                    twist.linear.x ,twist.linear.y, twist.linear.z = 0.0,0.0,0.0
                                    self.publisher_.publish(twist)
                                    # self.home_dock =  True
                            #   if abs(linear_home_vel)<0.01:
                            #        print('goal aa gya')
                            #        self.home_aligned = True
                            #        twist.angular.x , twist.angular.y ,twist.angular.z = 0.0, 0.0, 0.0
                            #        twist.linear.x ,twist.linear.y, twist.linear.z = 0.0,0.0,0.0
                            #        self.publisher_.publish(twist)
                                # if(self.home_dock == True): 
                                    
                        else:
                                self.dock_aligned_translate = True
                                twist.angular.x , twist.angular.y ,twist.angular.z = 0.0, 0.0, 0.0
                                twist.linear.x ,twist.linear.y, twist.linear.z = 0.0,0.0,0.0
                    self.publisher_.publish(twist)
            
            

    # Callback function for the DockControl service
    def dock_control_callback(self, request, response):
        # Extract desired docking parameters from the service request
        self.linear_dock = request.linear_dock
        self.orientation_dock = request.orientation_dock
        self.distance = request.distance
        self.orientation = request.orientation
        self.rack_no = request.rack_no
        self.start = True
        print(request)
        #


        # Reset flags and start the docking process
        self.is_docking = True
        self.dock_aligned_angle = False
        self.dock_aligned_translate = False
        #

        # Log a message indicating that docking has started
        self.get_logger().info("Docking started!")

        # Create a rate object to control the loop frequency
        rate = self.create_rate(2, self.get_clock())

        # Wait until the robot is aligned for docking
       
        while not self.dock_aligned_translate:
            self.get_logger().info("Waiting for alignment...")
            rate.sleep()

# Set the service response indicating success
        response.success = True
        response.message = "Docking control initiated"
        return response

# Main function to initialize the ROS2 node and spin the executor
def main(args=None):
    rclpy.init(args=args)

    my_robot_docking_controller = MyRobotDockingController()

    executor = MultiThreadedExecutor()
    executor.add_node(my_robot_docking_controller)

    executor.spin()

    my_robot_docking_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
