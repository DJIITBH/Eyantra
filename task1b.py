#!/usr/bin/env python3
"""
Example of moving to a pose goal.
`ros2 run pymoveit2 ex_pose_goal.py --ros-args -p position:="[0.25, 0.0, 1.0]" -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]" -p cartesian:=False`
"""

from threading import Thread
from os import path
import rclpy
import sys
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
import os
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
import time
from linkattacher_msgs.srv import AttachLink
from linkattacher_msgs.srv import DetachLink
from example_interfaces.msg import String

import numpy as np
from math import cos, sin
import math, time
from copy import deepcopy
import rclpy
import tf2_ros
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from pymoveit2.robots import ur5
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

DEFAULT_EXAMPLE_MESH = path.join(
    path.dirname(path.realpath(__file__)), "assets", "rack2.stl"
)


class ArmControl(Node):
    def __init__(self):
        super().__init__("arm_control")
        self.node=self
        self.subscriber_=self.create_subscription(String,"car_update",self.callback_arm_update,10)
        
        self.subscriber_=None
        self.callback_group = ReentrantCallbackGroup()
        self.moveit2 = MoveIt2(
            node=self.node,
            joint_names=ur5.joint_names(),
            base_link_name=ur5.base_link_name(),
            end_effector_name=ur5.end_effector_name(),
            group_name=ur5.MOVE_GROUP_ARM,
            callback_group=self.callback_group,
        )
        self.start =None
        self.i = 0
        self.tf_buffer = Buffer()                                                                       # initializing transform buffer object
        self.tf_listener = TransformListener(self.tf_buffer, self)                                      # initializing transform listner object
        self.timer = self.create_timer(1.0, self.on_timer)
        self.__twist_msg = TwistStamped()
        self.__twist_msg.header.frame_id = ur5.base_link_name()
        self.__twist_msg.twist.linear.x = 1.0
        self.__twist_msg.twist.linear.y = 1.0
        self.__twist_msg.twist.linear.z = 1.0
        self.__twist_msg.twist.angular.x = 1.0
        self.__twist_msg.twist.angular.y = 1.0
        self.__twist_msg.twist.angular.z = 1.0
        # self.create_timer(0.02, self.servo_circular_motion)
        callback_group = ReentrantCallbackGroup()
        self.__twist_pub = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.obj = []
        
        
        
        # self.objects = ['box3'] #idhar edit krna hai!
        self.executor_ = rclpy.executors.MultiThreadedExecutor(2)
        self.executor_.add_node(self)
        self.executor_thread = Thread(target=self.executor_.spin, daemon=True, args=())
        self.executor_thread.start()
        self.gripper_control_1 = self.create_client(AttachLink, '/GripperMagnetON')
        self.gripper_control_2 = self.create_client(DetachLink, '/GripperMagnetOFF')
        while not self.gripper_control_1.wait_for_service(timeout_sec=1.0):
         self.get_logger().info('EEF service not available, waiting again...')
        while not self.gripper_control_2.wait_for_service(timeout_sec=1.0):
         self.get_logger().info('EEF service not available, waiting again...')
        # # FOr move_to_position
        
        
        # For adding mesh
        self.node.declare_parameter(
        "filepath",
        "",
        )
        self.node.declare_parameter(
        "action",
        "add",
        )

        self.node.declare_parameter("position_1", [0.25,0.74,-0.55])
        self.node.declare_parameter("quat_xyzw_1", [0.00, 0.00, 0.7173561, 0.6967067])

        self.node.declare_parameter("position_2", [0.54,0.05,-0.56])
        self.node.declare_parameter("quat_xyzw_2", [0.0, 0.0, 0.0, 1.0])

        self.node.declare_parameter("position_3", [0.24,-0.63,-0.56])
        self.node.declare_parameter("quat_xyzw_3", [0.0, 0.0, 0.7173561, 0.6967067])


        self.filepath = self.node.get_parameter("filepath").get_parameter_value().string_value

        self.action = self.node.get_parameter("action").get_parameter_value().string_value
        self.position_1 = self.node.get_parameter("position_1").get_parameter_value().double_array_value
        self.quat_xyzw_1 = self.node.get_parameter("quat_xyzw_1").get_parameter_value().double_array_value

        self.position_2 = self.node.get_parameter("position_2").get_parameter_value().double_array_value
        self.quat_xyzw_2 = self.node.get_parameter("quat_xyzw_2").get_parameter_value().double_array_value

        self.position_3 = self.node.get_parameter("position_3").get_parameter_value().double_array_value
        self.quat_xyzw_3 = self.node.get_parameter("quat_xyzw_3").get_parameter_value().double_array_value

        if not (self.filepath):
            self.node.get_logger().info(f"Using the default example mesh file")
            self.filepath = DEFAULT_EXAMPLE_MESH

        if not path.exists(self.filepath):
            self.node.get_logger().error(f"File '{self.filepath}' does not exist")
            rclpy.shutdown()
            exit(1)

        #self.action = 'add'
    def callback_arm_update(self,msg):
        arm_flag = msg.data
        self.start = arm_flag
        print(arm_flag)

    def callback_obj_data(self,msg):
            if self.start == 'True':
                self.get_logger().info(str(msg.data))
                self.obj = eval(msg.data)
                if self.i ==0:
                    if type(self.obj) == int:
                        self.obj = [self.obj]
                    self.objects = []
                    for item in self.obj:
                        self.objects.append(f'box{item}')
                    self.i = 1
        #    print(type(eval(msg.data)))

    def on_timer(self):
        
        from_frame_rel = []
        if type(self.obj) == int:
            self.obj = [self.obj]
        for item in self.obj:
            from_frame_rel.append(f'2022_obj_{item}')

        # from_frame_rel = ['2022_obj_3' ]          #isko edit krna hai                                                           # frame from which transfrom has been sent
        to_frame_rel = 'base_link'                                                                      # frame to which transfrom has been sent
        self.data_ = []
        try:
            for element in from_frame_rel:
                t = self.tf_buffer.lookup_transform( to_frame_rel, element, rclpy.time.Time())       # look up for the transformation between 'obj_1' and 'base_link' frames
                self.data_.append([[t.transform.translation.x, t.transform.translation.y, t.transform.translation.z], [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]])
                # self.get_logger().info(f'Successfully received data!')
            
        except TransformException as e:
            self.get_logger().info(f'Could not transform {to_frame_rel} to {from_frame_rel}: {e}')
            return  

    def attach(self,name):
        req = AttachLink.Request()
        req.model1_name =  name     
        req.link1_name  = 'link'       
        req.model2_name =  'ur5'       
        req.link2_name  = 'wrist_3_link'  
        self.gripper_control_1.call_async(req)
        # os.system(f'ros2 service call /GripperMagnetON linkattacher_msgs/srv/AttachLink "{f"model1_name: {name}, link1_name: link, model2_name: ur5, link2_name: wrist_3_link"}"')

    def dettach(self,name):
        req = DetachLink.Request()
        req.model1_name =  name     
        req.link1_name  = 'link'       
        req.model2_name =  'ur5'       
        req.link2_name  = 'wrist_3_link'  
        self.gripper_control_2.call_async(req)

    def servoing_forward(self, item):
        if item == 0:
            start = time.time()
            while time.time()-start<=6.5: #1.7
                self.servo_circular_motion(0.4,0.0,0.0)
        elif item == 1:
            start = time.time()
            while time.time()-start<=1.7:
                self.servo_circular_motion(0.4,0.40,0.0)
        elif item == 2:
            start = time.time()
            while time.time()-start<=1.7:
                self.servo_circular_motion(0.1,-0.30,0.0)


    def servoing_backward(self, item):
        if item == 0:
            start = time.time()
            # if np.count_nonzero(np.array(self.ls) == 0) > 1:

            #     if self.data[i][0][2] > 0:
            #         while time.time()-start<=1.5:
            #             self.servo_circular_motion(-0.5,-0.3,+0.5)
            #     else:
            #         while time.time()-start<=1.5:
            #             self.servo_circular_motion(-0.5,0.2,+0.4)
            # else:
            while time.time()-start<=4.0:
                    self.servo_circular_motion(-2.0,0.0,0.0)
        elif item == 1:
            start = time.time()
            while time.time()-start<=1.4:
                self.servo_circular_motion(0.0,-0.60,0.0) #-0.2,-0.6,-0.4
        elif item == 2:
            start = time.time()
            while time.time()-start<=1.4:
                self.servo_circular_motion(-0.5,+0.60,0.0) #-0.5 in x




    def move_one(self):
        """
            Center - 0
            left - 1
            right - 2
        """
        ls = []
        for i in range(len(self.data)):
            if self.data[i][0][1] > 0.3:
                self.node.declare_parameter(f"position_{i+1}{i+1}", [self.data[i][0][0],self.data[i][0][1]-0.1,self.data[i][0][2]])
                self.node.declare_parameter(f"quat_xyzw_{i+1}{i+1}", [self.data[i][1][0],self.data[i][1][1],self.data[i][1][2],self.data[i][1][3]])
                self.node.declare_parameter(f"cartesian_{i+1}{i+1}", False)
                ls.append(1)
            elif self.data[i][0][1] < -0.2:
                self.node.declare_parameter(f"position_{i+1}{i+1}", [self.data[i][0][0],self.data[i][0][1]+0.07,self.data[i][0][2]])
                self.node.declare_parameter(f"quat_xyzw_{i+1}{i+1}", [self.data[i][1][0],self.data[i][1][1],self.data[i][1][2],self.data[i][1][3]])
                self.node.declare_parameter(f"cartesian_{i+1}{i+1}", False)
                ls.append(2)
            else:
                self.node.declare_parameter(f"position_{i+1}{i+1}", [self.data[i][0][0]-0.10,self.data[i][0][1],self.data[i][0][2]])
                self.node.declare_parameter(f"quat_xyzw_{i+1}{i+1}", [self.data[i][1][0],self.data[i][1][1],self.data[i][1][2],self.data[i][1][3]])
                self.node.declare_parameter(f"cartesian_{i+1}{i+1}", False)
                ls.append(0)
        return ls
            


    def move_to_position(self):
        # time.sleep(2)
        self.data = self.data_
        # print(self.data)
        # self.node.declare_parameter("position_11", [self.data[0][0][0]-0.1,self.data[0][0][1],self.data[0][0][2]])
        # #self.node.declare_parameter("quat_xyzw_11", [0.5,0.5,0.5,0.5])
        # #self.node.declare_parameter("quat_xyzw_11", [0.57530829, 0.41160422, 0.57486728, 0.41125411]) #perfect position
        # self.node.declare_parameter("quat_xyzw_11", [self.data[0][1][0],self.data[0][1][1],self.data[0][1][2],self.data[0][1][3]])
        # self.node.declare_parameter("cartesian_11", False)

        # self.node.declare_parameter("position_15", [self.data[0][0][0],self.data[0][0][1],self.data[0][0][2]])
        # #self.node.declare_parameter("quat_xyzw_11", [0.5,0.5,0.5,0.5])
        # #self.node.declare_parameter("quat_xyzw_11", [0.57530829, 0.41160422, 0.57486728, 0.41125411]) #perfect position
        # self.node.declare_parameter("quat_xyzw_15", [self.data[0][1][0],self.data[0][1][1],self.data[0][1][2],self.data[0][1][3]])
        # self.node.declare_parameter("cartesian_15", False)
        

        self.node.declare_parameter(
        "joint_positions_1",
        [
            -0.052,
            -2.30,
            -0.34,
            -3.60,
            -1.57,
            3.130
        ],
      )
        self.node.declare_parameter(
        "joint_positions_2",
        [
            0.000,
            -2.3900,
            2.400,
            -3.1500,
            -1.5799,
            3.1500
        ],
      )
        
        self.node.declare_parameter(
        "joint_positions_3",
        [
            -1.57,
            -2.3900,
            2.400,
            -3.1500,
            -1.5799,
            3.1500
        ],
    )
        self.node.declare_parameter(
        "joint_positions_4",
        [
            1.40,
            -2.3900,
            2.400,
            -3.1500,
            -1.5799,
            3.1500
        ],
    )

        # self.node.declare_parameter("position_22", [-0.37, 0.12, 0.397])
        # self.node.declare_parameter("quat_xyzw_22", [0.57530829, 0.41160422, 0.57486728, 0.41125411])
        # self.node.declare_parameter("cartesian_22", False)

        # self.node.declare_parameter("position_33", [self.data[1][0][0],self.data[1][0][1],self.data[1][0][2]])
        # #self.node.declare_parameter("quat_xyzw_33", [0.0, -0.0022565, -0.7004294, 0.7137181])
        # #self.node.declare_parameter("quat_xyzw_33", [0.70710678, 0.70710678, 0.00000000, 0.00000000]) #working
        # #self.node.declare_parameter("quat_xyzw_33", [0.70762968, -0.00289776, -0.70657163,  0.00289343]) #also working
        # self.node.declare_parameter("quat_xyzw_33", [self.data[1][1][0],self.data[1][1][1],self.data[1][1][2],self.data[1][1][3]]) #calc from quaternion
        # #self.node.declare_parameter("quat_xyzw_33", [0.72749677, 0.67277624, -0.08952772, 0.10052543])
        # self.node.declare_parameter("cartesian_33", False)

        # self.node.declare_parameter("position_44", [self.data[2][0][0],self.data[2][0][1],self.data[2][0][2]])
        # self.node.declare_parameter("quat_xyzw_44", [self.data[2][1][0],self.data[2][1][1],self.data[2][1][2],self.data[2][1][3]])
        # self.node.declare_parameter("cartesian_44", False)
        #  wapas krlio
        joint_positions_1 = (
        self.node.get_parameter("joint_positions_1").get_parameter_value().double_array_value
        ) 

        joint_positions_2= (
        self.node.get_parameter("joint_positions_2").get_parameter_value().double_array_value
        )

        joint_positions_3= (
        self.node.get_parameter("joint_positions_3").get_parameter_value().double_array_value
        )

        joint_positions_4= (
        self.node.get_parameter("joint_positions_4").get_parameter_value().double_array_value
        )
        
        self.ls = self.move_one()
        position = []
        start = time.time()
        # while time.time() - start < 1:
        #     self.servo_circular_motion(0.0, 0.0, 0.5)
            #loop is commented for now
        # for i in range(len(ls)):
        #     position = self.node.get_parameter(f"position_{i+1}{i+1}").get_parameter_value().double_array_value
        #     quat_xyzw = self.node.get_parameter(f"quat_xyzw_{i+1}{i+1}").get_parameter_value().double_array_value
        #     cartesian = self.node.get_parameter(f"cartesian_{i+1}{i+1}").get_parameter_value().bool_value

        #     self.moveit2.move_to_configuration(joint_positions_2)
        #     self.moveit2.wait_until_executed()


        #     self.moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw, cartesian=cartesian)
        #     self.moveit2.wait_until_executed()

        #     self.servoing_forward(ls[i])
        #     self.attach(self.objects[i])
        #     self.servoing_backward(ls[i])

        #     if ls[i] == 0:
        #         self.moveit2.move_to_configuration(joint_positions_2)
        #         self.moveit2.wait_until_executed()



        #     self.moveit2.move_to_configuration(joint_positions_1)
        #     self.moveit2.wait_until_executed()

        #     self.dettach(self.objects[i])

        #looping here #
        position_box = []
        quat_xyzw_box = []
        cartesian_box = []
        for i in range(len(self.ls)):
            position_box.append(self.node.get_parameter(f"position_{i+1}{i+1}").get_parameter_value().double_array_value)
            quat_xyzw_box.append(self.node.get_parameter(f"quat_xyzw_{i+1}{i+1}").get_parameter_value().double_array_value)
            cartesian_box.append(self.node.get_parameter(f"cartesian_{i+1}{i+1}").get_parameter_value().bool_value)

    # #     position_15 = self.node.get_parameter("position_11").get_parameter_value().double_array_value
    # #     quat_xyzw_15 = self.node.get_parameter("quat_xyzw_11").get_parameter_value().double_array_value
    # #     cartesian_15 = self.node.get_parameter("cartesian_11")
    # .get_parameter_value().bool_value


         

        # position_22 = self.node.get_parameter("position_22").get_parameter_value().double_array_value
        # quat_xyzw_22 = self.node.get_parameter("quat_xyzw_22").get_parameter_value().double_array_value
        # cartesian_22 = self.node.get_parameter("cartesian_22").get_parameter_value().bool_value

        # position_44 = self.node.get_parameter("position_44").get_parameter_value().double_array_value
        # quat_xyzw_44 = self.node.get_parameter("quat_xyzw_44").get_parameter_value().double_array_value
        # cartesian_44 = self.node.get_parameter("cartesian_44").get_parameter_value().bool_value

        # position_33 = self.node.get_parameter("position_33").get_parameter_value().double_array_value
        # quat_xyzw_33 = self.node.get_parameter("quat_xyzw_33").get_parameter_value().double_array_value
        # cartesian_33 = self.node.get_parameter("cartesian_33").get_parameter_value().bool_value
        
        # self.moveit2.move_to_configuration(joint_positions_2)
        # self.moveit2.wait_until_executed()

        # start1 = time.time()
        # while time.time()-start1<1:
        #     # self.servo_motion()
        #     self.servo_circular_motion(0.0,0.0,0.2)

    #     self.node.get_logger().info(
    #     f"Moving to {{position: {list(position_11)}, quat_xyzw: {list(quat_xyzw_11)}}}"


    #     )

        # start1 = time.time()
        # while time.time()-start1<0.2:
        #     # self.servo_motion()
        #     self.servo_circular_motion(0.0,0.0,0.1)
        # if(self.ls[0]==0 and np.count_nonzero(np.array(self.ls)==0)==1):
        #     start = time.time()
        #     while time.time()-start<0.7:
        #         self.servo_motion(0)
        # else:

        # start1 = time.time()
        # while time.time()-start1<0.5:
        #     # self.servo_motion()
        #     self.servo_circular_motion(0.0,0.0,0.1)
        for i in range(len(self.ls)):
            if self.ls[i]==0:
                self.moveit2.move_to_configuration(joint_positions_2)
                self.moveit2.wait_until_executed()
                
            elif self.ls[i]==1:
                self.moveit2.move_to_configuration(joint_positions_4)
                self.moveit2.wait_until_executed()
               
            else:
                self.moveit2.move_to_configuration(joint_positions_3)
                self.moveit2.wait_until_executed()
                
            start = time.time()

            # while time.time()-start<3: #1.5
            #     self.servo_motion(i)
            self.moveit2.move_to_pose(position=position_box[i], quat_xyzw=quat_xyzw_box[i], cartesian=cartesian_box[i])
            self.moveit2.wait_until_executed()
            self.servoing_forward(self.ls[i])
            self.attach(self.objects[i])
            self.servoing_backward(self.ls[i])
            
                # self.moveit2.move_to_configuration(joint_positions_3)
                # self.moveit2.wait_until_executed()
            self.moveit2.move_to_configuration(joint_positions_2)
            self.moveit2.wait_until_executed()
            # start1 = time.time()
            # while time.time()-start1<1:
            #     # self.servo_motion()
            #     self.servo_circular_motion(-3.0,0.2,0.4)
            self.moveit2.move_to_configuration(joint_positions_1)
            self.moveit2.wait_until_executed()
            self.dettach(self.objects[i])
        
        # start1 = time.time()
        # while time.time()-start1<0.2:
        #     # self.servo_motion()
        #     self.servo_circular_motion(0.2,0.0,0.0)
        # # time.sleep(1)

        # self.moveit2.move_to_configuration(joint_positions_2)
        # self.moveit2.wait_until_executed()

        # if(self.ls[0]==0 and np.count_nonzero(np.array(self.ls)==0)==1):
        #     start = time.time()
        #     while time.time()-start<0.7:
        #         self.servo_motion(1)
        # else:
        # self.moveit2.move_to_configuration(joint_positions_2)
        # self.moveit2.wait_until_executed()
        # # start1 = time.time()
        # # while time.time()-start1<0.5:
        # #     # self.servo_motion()
        # #     self.servo_circular_motion(0.0,0.0,0.1)  
        # if self.ls[1]==0:
        #     self.moveit2.move_to_configuration(joint_positions_2)
        #     self.moveit2.wait_until_executed()
        #     tim = 3
        # elif self.ls[1]==1:
        #     self.moveit2.move_to_configuration(joint_positions_4)
        #     self.moveit2.wait_until_executed()
        #     tim = 4.5
        # else:
        #     self.moveit2.move_to_configuration(joint_positions_3)
        #     self.moveit2.wait_until_executed()
        #     tim = 4.5
        # start = time.time()
        # while time.time()-start<1.5:
        #     self.servo_motion(1)
        # self.servoing_forward(self.ls[1])
        # self.attach(self.objects[1])
        # self.servoing_backward(self.ls[1])
    
        #     # self.moveit2.move_to_configuration(joint_positions_3)
        #     # self.moveit2.wait_until_executed()
        # self.moveit2.move_to_configuration(joint_positions_2)
        # self.moveit2.wait_until_executed()
        # # self.moveit2.move_to_configuration(joint_positions_2)
        # # self.moveit2.wait_until_executed()
        # # while time.time()-start1<1:
        # #     # self.servo_motion()
        # #     self.servo_circular_motion(-3.0,0.2,0.4)
        # self.moveit2.move_to_configuration(joint_positions_1)
        # self.moveit2.wait_until_executed()
        # self.dettach(self.objects[1])
        # # while time.time()-start1<0.2:
        # #     # self.servo_motion()
        # #     self.servo_circular_motion(0.2,0.0,0.1)
        # # # time.sleep(1)

       

        # # start1 = time.time()
        # # while time.time()-start1<0.2:
        # #     # self.servo_motion()
        # #     self.servo_circular_motion(0.0,0.0,0.1)
        # # if(self.ls[0]==0 and np.count_nonzero(np.array(self.ls)==0)==1):
        # #     start = time.time()
        # #     while time.time()-start<0.7:
        # #         self.servo_motion(2)
        # # else:
        # self.moveit2.move_to_configuration(joint_positions_2)
        # self.moveit2.wait_until_executed()
        # # start1 = time.time()
        # # while time.time()-start1<0.5:
        # #     # self.servo_motion()
        # #     self.servo_circular_motion(0.0,0.0,0.1)  

       
        # # self.moveit2.move_to_pose(position=position_33, quat_xyzw=quat_xyzw_33, cartesian=cartesian_33)
        # # self.moveit2.wait_until_executed()
        # if self.ls[2]==0:
        #     self.moveit2.move_to_configuration(joint_positions_2)
        #     self.moveit2.wait_until_executed()
        #     tim = 2
        # elif self.ls[2]==1:
        #     self.moveit2.move_to_configuration(joint_positions_4)
        #     self.moveit2.wait_until_executed()

        # else:
        #     self.moveit2.move_to_configuration(joint_positions_3)
        #     self.moveit2.wait_until_executed()
        #     tim = 4.5
        # start = time.time()
        # while time.time()-start<1.5:
        #     self.servo_motion(2)
        # self.servoing_forward(self.ls[2])
        # self.attach(self.objects[2])
        # self.servoing_backward(self.ls[2])
       
        #     # self.moveit2.move_to_configuration(joint_positions_3)
        #     # self.moveit2.wait_until_executed()
        # self.moveit2.move_to_configuration(joint_positions_2)
        # self.moveit2.wait_until_executed()
        # # self.moveit2.move_to_configuration(joint_positions_2)
        # # self.moveit2.wait_until_executed()
        # start1 = time.time()
        # # while time.time()-start1<1:
        # #     # self.servo_motion()
        # #     self.servo_circular_motion(-3.0,0.2,0.4)
        # self.moveit2.move_to_configuration(joint_positions_1)
        # self.moveit2.wait_until_executed()
        # self.dettach(self.objects[2])

        # self.moveit2.move_to_configuration(joint_positions_2)
        # self.moveit2.wait_until_executed()
        
    #     start = time.time()
    #     # while time.time()-start<1:
    #     #     self.servo_circular_motion(0.3,0.00,0.00)
    #     # self.attach('box49')

    #     start = time.time()
    #     # while time.time()-start<1.7:
    #     #     self.servo_circular_motion(-0.6,0.00,0.00)
    #     start = time.time()
    #     # while time.time()-start<1:
    #     #     self.servo_circular_motion(0.00,0.00,0.5)

  
    def servo_motion(self,j):
        """Move in a circular motion using Servo"""
        self.data = self.data_
        # print(self.data )
        x=self.data[j][0][0]
        y=self.data[j][0][1]
        z=self.data[j][0][2]
        d = math.sqrt(x**2+y**2+z**2)
        linear = [x/d,y/d,z/d]
        # if k==0:
        #     linear = [(x-0.08)/d,(y-0.109)/d,(z-0.47)/d]
        angular = [0.0,0.0,0.0]
        twist_msg = deepcopy(self.__twist_msg)
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        #print("chu",twist_msg.twist.linear.x)
        twist_msg.twist.linear.x=linear[0]*(1/2)
        twist_msg.twist.linear.y=linear[1]*(1/2)
        twist_msg.twist.linear.z=linear[2]*(1/2)
        twist_msg.twist.angular.x=0.0
        twist_msg.twist.angular.y=0.0
        twist_msg.twist.angular.z=0.0
        # print(twist_msg)
        self.__twist_pub.publish(twist_msg)
    def servo_circular_motion(self,vx,vy,vz):
        """Move in a circular motion using Servo"""
        # self.data = self.data_
        # print(self.data )
        # x=self.data[0][0][0]
        # y=self.data[0][0][1]
        # z=self.data[0][0][2]
        # d = math.sqrt(x**2+y**2+z**2)
        # linear = [x/d,y/d,z/d]
        # angular = [0.0,0.0,0.0]
        twist_msg = deepcopy(self.__twist_msg)
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        #print("chu",twist_msg.twist.linear.x)
        twist_msg.twist.linear.x=vx
        twist_msg.twist.linear.y=vy
        twist_msg.twist.linear.z=vz
        twist_msg.twist.angular.x=0.0
        twist_msg.twist.angular.y=0.0
        twist_msg.twist.angular.z=0.0
        # print(twist_msg)
        self.__twist_pub.publish(twist_msg)
        # self.create_timer(0.02, self.servo_circular_motion)


        
    def add_mesh(self):
        mesh_id_1 = 'rack1'
        mesh_id_2 = 'rack2'
        mesh_id_3 = 'rack3'
        if "add" == self.action:
            # Add collision mesh
            self.node.get_logger().info(
                f"Adding collision mesh 1 '{self.filepath}' {{position: {list(self.position_1)}, quat_xyzw: {list(self.quat_xyzw_1)}}}"
            )
            # print(ur5.base_link_name())
            self.moveit2.add_collision_mesh(
                filepath=self.filepath, id=mesh_id_1, position=self.position_1, quat_xyzw=self.quat_xyzw_1, frame_id=ur5.base_link_name()
            )
            self.moveit2.add_collision_mesh(
                filepath=self.filepath, id=mesh_id_2, position=self.position_2, quat_xyzw=self.quat_xyzw_2, frame_id=ur5.base_link_name()
            )
            self.moveit2.add_collision_mesh(
                filepath=self.filepath, id=mesh_id_3, position=self.position_3, quat_xyzw=self.quat_xyzw_3, frame_id=ur5.base_link_name()
            )
        else:
            # Remove collision mesh
            self.node.get_logger().info(f"Removing collision mesh with ID '{mesh_id_1}'")
            self.moveit2.remove_collision_mesh(id=mesh_id_1)
            self.moveit2.remove_collision_mesh(id=mesh_id_2)
            self.moveit2.remove_collision_mesh(id=mesh_id_3)

        


    


def main():

    '''
    Description:    Main function which creates a ROS node and spin around for the aruco_tf class to perform it's task
    '''

    rclpy.init(args=None)                                       # initialisation

    # node = rclpy.create_node('arm_control')                    # creating ROS node
    armcontrol = ArmControl()
    # rate = armcontrol.create_rate(2, armcontrol.get_clock())
    while armcontrol.start == 'False' or armcontrol.start == None:
        armcontrol.get_logger().info("Waiting for True value...")
    armcontrol.subscriber_=armcontrol.create_subscription(String,"objects_data",armcontrol.callback_obj_data,10)
    print('\n\n\n\n\n\n\n\n\n\n\n\n\n')
    armcontrol.get_logger().info('Node created: Armcontrol')   

    armcontrol.on_timer()     # logging information 
    armcontrol.add_mesh()                                 # creating a new object for class 'aruco_tf'
    # start = time.time()
    # while time.time()-start<0.6:
    #     armcontrol.servo_circular_motion()
    armcontrol.move_to_position()
   
    # armcontrol.servo_circular_motion()
    # rclpy.spin(ArmControl_class)                                      # spining on the object to make it alive in ROS 2 DDS

    armcontrol.destroy_node()                                   # destroy node after spin ends

    rclpy.shutdown()                                                # shutdown process


if __name__ == "__main__":
    main()
