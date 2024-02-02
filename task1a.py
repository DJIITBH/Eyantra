#!/usr/bin/env python3


'''
*****************************************************************************************
*
*        		===============================================
*           		    Cosmo Logistic (CL) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script should be used to implement Task 1A of Cosmo Logistic (CL) Theme (eYRC 2023-24).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:          [ CL#2022 ]
# Author List:		[Dhananjay Abbot, Bhavik Shangari, Pushan Kumar]
# Filename:		    task1a.py
# Functions:
#			        [ calculate_rectangle_area(), detect_aruco(), quaternion_from_euler(), quaternion_multiply(), process_image(), __init__(), publish_tf(), depthimagecb(), colorimagecb(). main() ]
# Nodes:		    Add your publishing and subscribing node
#                   Example:
#			        Publishing Topics  - [ /tf ]
#                   Subscribing Topics - [ /camera/aligned_depth_to_color/image_raw, /etc... ]


################### IMPORT MODULES #######################

import rclpy
import sys
import cv2
import math
import cv2.aruco as aruco
import numpy as np
import tf2_ros
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CompressedImage, Image
import tf2_py
import time
from tf2_ros import TransformBroadcaster
from example_interfaces.msg import String

camera_matrix = np.array([[931.1829833984375, 0.0, 640.0], [0.0, 931.1829833984375, 360.0], [0.0, 0.0, 1.0]])
dist_coeffs = np.array([0.0,0.0,0.0,0.0,0.0])

# [[455.60356641   0.         304.98133381]
#  [  0.         458.10488942 186.28803718]
#  [  0.           0.           1.        ]] [[-0.10660093 -0.12610379 -0.01145278 -0.00680098  0.14284421]]
##################### FUNCTION DEFINITIONS #######################

def calculate_rectangle_area(coordinates):
    '''
    Description:    Function to calculate area or detected aruco

    Args:
        coordinates (list):     coordinates of detected aruco (4 set of (x,y) coordinates)

    Returns:
        area        (float):    area of detected aruco
        width       (float):    width of detected aruco
    '''

    ############ Function VARIABLES ############

    # You can remove these variables after reading the instructions. These are just for sample.

    area = None
    width = None

    ############ ADD YOUR CODE HERE ############

    # INSTRUCTIONS & HELP : 
    #	->  Recevice coordiantes from 'detectMarkers' using cv2.aruco library 
    #       and use these coordinates to calculate area and width of aruco detected.
    #	->  Extract values from input set of 4 (x,y) coordinates 
    #       and formulate width and height of aruco detected to return 'area' and 'width'.

    ############################################
    if coordinates is not None:
        length = math.sqrt( math.pow( coordinates[0][0] - coordinates[1][0] , 2 ) + math.pow( coordinates[0][1] - coordinates[1][1] , 2 ))
        bredth = math.sqrt( math.pow( coordinates[1][0] - coordinates[2][0] , 2 ) + math.pow( coordinates[1][1] - coordinates[2][1] , 2 ))

        area = length * bredth

        if length == bredth :
            width = bredth
        elif length > bredth :
            width = bredth
        else :
            width = length


    return area, width



def detect_aruco(image):
    '''
    Description:    Function to perform aruco detection and return each detail of aruco detected 
                    such as marker ID, distance, angle, width, center point location, etc.

    Args:
        image                   (Image):    Input image frame received from respective camera topic

    Returns:
        center_aruco_list       (list):     Center points of all aruco markers detected
        distance_from_rgb_list  (list):     Distance value of each aruco markers detected from RGB camera
        angle_aruco_list        (list):     Angle of all pose estimated for aruco marker
        width_aruco_list        (list):     Width of all detected aruco markers
        ids                     (list):     List of all aruco marker IDs detected in a single frame 
    '''

    ############ Function VARIABLES ############

    # ->  You can remove these variables if needed. These are just for suggestions to let you get started

    # Use this variable as a threshold value to detect aruco markers of certain size.
    # Ex: avoid markers/boxes placed far away from arm's reach position  
    aruco_area_threshold = 2000

    # The camera matrix is defined as per camera info loaded from the plugin used. 
    # You may get this from /camer_info topic when camera is spawned in gazebo.
    # Make sure you verify this matrix once if there are calibration issues.
    # camera_matrix = np.array([[931.1829833984375, 0.0, 640.0], [0.0, 931.1829833984375, 360.0], [0.0, 0.0, 1.0]])

    # The distortion matrix is currently set to 0. 
    # We will be using it during Stage 2 hardware as Intel Realsense Camera provides these camera info.
    # dist_coeffs = np.array([0.0,0.0,0.0,0.0,0.0])
    # calbration_data = np.load('/src/ur_description/scripts/calibration.npz')
    # camera_matrix = np.array([[455.60356641  , 0. ,        304.98133381],
    #     [  0.     ,    458.10488942 ,186.28803718],
    #     [  0.   ,        0.        ,   1.        ]])
    
    # dist_coeffs = np.array([-0.10660093 ,-0.12610379, -0.01145278, -0.00680098,  0.14284421])

    # We are using 150x150 aruco marker size
    marker_size = 0.15
    # You can remove these variables after reading the instructions. These are just for sample.
    center_aruco_list = []
    distance_from_rgb_list = []
    angle_aruco_list = []
    width_aruco_list = []
    ids = []
    areas=[]
    index = []
    try:
        parameters = cv2.aruco.DetectorParameters()
        #print('hi')
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    #   ->  Use these aruco parameters-
    #       ->  Dictionary: 4x4_50 (4x4 only until 50 aruco IDs)
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
        #   ->  Detect aruco marker in the image and store 'corners' and 'ids'
        #       ->  HINT: Handle cases for empty markers detection. 
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

        corners, ids, _ = detector.detectMarkers(gray)
        # print('before', corners.shape)

        for i in range(len(ids)):
            cur_corner = [corners[i]]
            aruco.drawDetectedMarkers(image, cur_corner, ids[i])

            ret = aruco.estimatePoseSingleMarkers(cur_corner, marker_size, camera_matrix, dist_coeffs)
            rvec ,tvec = ret[0][0, 0, :], ret[1][0, 0, :]
            x, y, z = tvec
            rot_matrix = cv2.Rodrigues(rvec.reshape(3, 1))[0]
            roll, pitch, yaw = cv2.RQDecomp3x3(rot_matrix)[0] 
            
            center_coor = [corners[i][0][0][0]+(corners[i][0][2][0]-corners[i][0][0][0])/2, corners[i][0][0][1]-(corners[i][0][0][1]-corners[i][0][2][1])/2]
            center_aruco_list.append(center_coor)
            angle_aruco_list.append([roll, pitch, yaw])
            area,width=calculate_rectangle_area(cur_corner[0][0])
            areas.append(area)
            width_aruco_list.append(width)
            if area<aruco_area_threshold:
                index.append(i)
            if area>aruco_area_threshold:
                cv2.drawFrameAxes(image, camera_matrix, dist_coeffs, rvec, tvec, length =1)
                cv2.putText(image,f"center",(int(center_coor[0]-25),int(center_coor[1]-20)),cv2.FONT_HERSHEY_PLAIN,1,(255,255,255),2,cv2.LINE_AA)
            # break
    #     if ids is not None:
    #         aruco.drawDetectedMarkers(image, corners, ids)  
    #         ids = list(ids)
    #         corners = list(corners)
    #     #   ->  Loop over each marker ID detected in frame and calculate area using function defined above (calculate_rectangle_area(coordinates))
    #         for i, id in enumerate(ids):
    #             area, width = (calculate_rectangle_area(corners[i]))
    #             areas.append(area)
    #             width_aruco_list.append(width)
    #     #   ->  Remove tags which are far away from arm's reach positon based on some threshold defined
    #         for j, area in enumerate(areas):
    #             if area < aruco_area_threshold:
    #                 ids.pop(j)
    #                 areas.pop(j)
    #                 corners.pop(j)
    #                 width_aruco_list.pop(j)
    #                 j=j-1
    #         ids = np.asarray(ids)
    #         corners = tuple(corners)
    
    #     # print(corners)
    #     #   ->  Draw detected marker on the image frame which will be shown later

    #     #   ->  Calculate center points aruco list using math and distance from RGB camera using pose estimation of aruco marker
    #     #       ->  HINT: You may use numpy for center points and 'estimatePoseSingleMarkers' from cv2 aruco library for pose estimation
    #     rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_size, cameraMatrix=camera_matrix, distCoeffs=dist_coeffs)
    #     for i in range(len(ids)):
    #         # Convert the rotation vector to Euler angles
    #         rvec_matrix = cv2.Rodrigues(rvecs[i])[0]
    #         angles = cv2.decomposeProjectionMatrix(np.hstack((rvec_matrix, tvecs[i].T)))[6]

    #         # Extract the roll, pitch, and yaw angles (in degrees)
    #         roll, pitch, yaw = angles[0], angles[1], angles[2]
    #         angle_aruco_list.append([roll, pitch, yaw])
    #         # Print the angles
    #         # print(f"Marker ID {ids[i][0]} - Roll: {roll} degrees, Pitch: {pitch} degrees, Yaw: {yaw} degrees")

    #         # Calculate the 3D world coordinates of the marker center
    #         marker_center_3d = tvecs[i][0]
            
    #         # Calculate the distance from the camera to the marker center
    #         distance = np.linalg.norm(marker_center_3d)
    #         distance_from_rgb_list.append(distance)
    # #   ->  Draw frame axes from coordinates received using pose estimation
            
    # #       ->  HINT: You may use 'cv2.drawFrameAxes'
    #         axis_length = 100.0  # Adjust the length as needed

    #         # Define the 3D coordinates of the axis endpoints in the frame's coordinate system
    #         axis_points = np.array([[0, 0, 0],  # Origin
    #                                 [axis_length, 0, 0],  # X-axis endpoint
    #                                 [0, axis_length, 0],  # Y-axis endpoint
    #                                 [0, 0, axis_length]])  # Z-axis endpoint
    #     # Project the 3D axis endpoints to 2D image coordinates
        
    #         image_points, _ = cv2.projectPoints(axis_points, rvecs[i], tvecs[i], camera_matrix, dist_coeffs)

    #         # Convert image points to integer values for drawing
    #         image_points = image_points.astype(int)

    #         # Draw the coordinate axes on an image
    #         # Replace 'frame' with your actual image
    #         origin = tuple(image_points[0].ravel())
    #         x_endpoint = tuple(image_points[1].ravel())
    #         y_endpoint = tuple(image_points[2].ravel())
    #         z_endpoint = tuple(image_points[3].ravel())
    #         center_aruco_list.append(origin)

    #         # Draw the axes lines
    #         cv2.line(image, origin, x_endpoint, (0, 0, 255), 2)  # X-axis (red)
    #         cv2.line(image, origin, y_endpoint, (0, 255, 0), 2)  # Y-axis (green)
    #         cv2.line(image, origin, z_endpoint, (255, 0, 0), 2)  # Z-axis (blue)
            # cv2.imshow('image', image)
            # cv2.waitKey(1)
        return center_aruco_list, angle_aruco_list, width_aruco_list, ids, index
        
    except Exception as e:
            print(e, 'in detect fxn')
            return None
    
    ############################################

    # return center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids
def quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.

    Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.

    Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return np.array([qx, qy, qz, qw])

def quaternion_multiply(q0, q1):
    """
    Multiplies two quaternions.

    Input
    :param q0: A 4 element array containing the first quaternion (q01, q11, q21, q31)
    :param q1: A 4 element array containing the second quaternion (q02, q12, q22, q32)

    Output
    :return: A 4 element array containing the final quaternion (q03,q13,q23,q33)

    """
    # Extract the values from q0
    w0 = q0[0]
    x0 = q0[1]
    y0 = q0[2]
    z0 = q0[3]

    # Extract the values from q1
    w1 = q1[0]
    x1 = q1[1]
    y1 = q1[2]
    z1 = q1[3]

    # Computer the product of the two quaternions, term by term
    q0q1_w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
    q0q1_x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
    q0q1_y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
    q0q1_z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1

    # Create a 4 element array containing the final quaternion
    final_quaternion = np.array([q0q1_w, q0q1_x, q0q1_y, q0q1_z])

    # Return a 4 element array containing the final quaternion (q02,q12,q22,q32)
    return final_quaternion


##################### CLASS DEFINITION #######################

class aruco_tf(Node):
    '''
    ___CLASS___

    Description:    Class which servers purpose to define process for detecting aruco marker and publishing tf on pose estimated.
    '''

    def __init__(self, node_name):
        '''
        Description:    Initialization of class aruco_tf
                        All classes have a function called __init__(), which is always executed when the class is being initiated.
                        The __init__() function is called automatically every time the class is being used to create a new object.
                        You can find more on this topic here -> https://www.w3schools.com/python/python_classes.asp
        '''

        super().__init__(node_name=node_name)                                          # registering node

        ############ Topic SUBSCRIPTIONS ############

        self.color_cam_sub = self.create_subscription(Image, '/camera/color/image_raw', self.colorimagecb, 10)
        self.depth_cam_sub = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depthimagecb, 10)
        self.publisher_=self.create_publisher(String,"objects_data",10)
        # self.timer_=self.create_timer(0.5,self.obj_data)
        

        ############ Constructor VARIABLES/OBJECTS ############

        image_processing_rate = 0.4                                                     # rate of time to process image (seconds)
        self.bridge = CvBridge()                                                        # initialise CvBridge object for image conversion
        self.tf_buffer = tf2_ros.buffer.Buffer()                                        # buffer time used for listening transforms
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)                                    # object as transform broadcaster to send transform wrt some frame_id
        self.timer = self.create_timer(image_processing_rate, self.process_image)       # creating a timer based function which gets called on every 0.2 seconds (as defined by 'image_processing_rate' variable)
        #self.timer = self.create_timer(1.0, self.publish_tf)
        self.cv_image = None                                                            # colour raw image variable (from colorimagecb())
        self.depth_image = None  
        self.clock = self.get_clock()                                                       # depth image variable (from depthimagecb())


    def depthimagecb(self, data):
        '''
        Description:    Callback function for aligned depth camera topic. 
                        Use this function to receive image depth data and convert to CV2 image

        Args:
            data (Image):    Input depth image frame received from aligned depth camera topic

        Returns:
        '''
        self.depth_image = self.bridge.imgmsg_to_cv2(data)
        self.depth_image = cv2.undistort(self.depth_image, camera_matrix, dist_coeffs)

        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP : 

        #	->  Use data variable to convert ROS Image message to CV2 Image type

        #   ->  HINT: You may use CvBridge to do the same

        ############################################
    def publish_tf(self, parent, child, translation, quaternion):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = parent
        tf_msg.child_frame_id = child
        tf_msg.transform.translation.x = translation[0]
        tf_msg.transform.translation.y = translation[1]
        tf_msg.transform.translation.z = translation[2]
        tf_msg.transform.rotation.x = quaternion[0]
        tf_msg.transform.rotation.y = quaternion[1]
        tf_msg.transform.rotation.z = quaternion[2]
        tf_msg.transform.rotation.w = quaternion[3]

        self.tf_broadcaster.sendTransform(tf_msg)

    def colorimagecb(self, data):
        '''
        Description:    Callback function for colour camera raw topic.
                        Use this function to receive raw image data and convert to CV2 image

        Args:
            data (Image):    Input coloured raw image frame received from image_raw camera topic

        Returns:
        '''
        self.cv_image = self.bridge.imgmsg_to_cv2(data)
        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP : 

        #	->  Use data variable to convert ROS Image message to CV2 Image type

        #   ->  HINT:   You may use CvBridge to do the same
        #               Check if you need any rotation or flipping image as input data maybe different than what you expect to be.
        #               You may use cv2 functions such as 'flip' and 'rotate' to do the same

        ############################################
    

    def process_image(self):
        '''
        Description:    Timer function used to detect aruco markers and publish tf on estimated poses.

        Args:
        Returns:
        '''

        ############ Function VARIABLES ############

        # These are the variables defined from camera info topic such as image pixel size, focalX, focalY, etc.
        # Make sure you verify these variable values once. As it may affect your result.
        # You can find more on these variables here -> http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html
        
        sizeCamX = 1280
        sizeCamY = 720
        centerCamX = 640 
        centerCamY = 360
        focalX = 931.1829833984375
        focalY = 931.1829833984375
            
        ############ ADD YOUR CODE HERE ############
        # detect_aruco(self.cv_image)
        # INSTRUCTIONS & HELP : 
        distance_from_depth_list = []
        
        #	->  Get aruco center, distance from rgb, angle, width and ids list from 'detect_aruco_center' defined above
        try:
            center_aruco_list, angle_aruco_list, width_aruco_list, ids, index = detect_aruco(self.cv_image)
            def obj_data():
                msg=String()
                a = []
                for i in range(len(ids)):
                    if i not in index:
                        a.append(ids[i])
                ls = str(np.array(a).squeeze().tolist())
                msg.data=ls

                # print(lo)
                self.publisher_.publish(msg) 
            obj_data()
            
            #   ->  Loop over detected box ids received to calculate position and orientation transform to publish TF 
            for i, id in enumerate(ids):
            #   ->  Use this equation to correct the input aruco angle received from cv2 aruco function 'estimatePoseSingleMarkers' here
            #       It's a correction formula-  
                if i in index:
                    continue

                depth = self.depth_image[int(center_aruco_list[i][1]), int(center_aruco_list[i][0])]/1000
                yaw = (0.788*angle_aruco_list[i][2]) - ((angle_aruco_list[i][2]**2)/3160)
                
            #   ->  Then calculate quaternions from roll pitch yaw (where, roll and pitch are 0 while yaw is corrected aruco_angle)
                # quaternion = tf2_ros.transformations.quaternion_from_euler(angle_aruco_list[i][0][0], angle_aruco_list[i][1][0], angle_aruco)
                # quaternion = quaternion_from_euler( angle_aruco_list[i][0],angle_aruco_list[i][1],angle_aruco)
                # quaternion = quaternion_from_euler(0,1.57,angle_aruco)
                
                # quaternion = quaternion_from_euler(0,1.57,yaw)

                # quaternion = quaternion_from_euler(4.71,3.14,-1.57+angle_aruco)
                # for i in range(3):
                #     if(center_aruco_list[i][0]>400 and center_aruco_list[i][0]<750):
                #         quaternion = quaternion_from_euler(1.57-0.278,0.05,1.57)
                #     elif (center_aruco_list[i][0]>750):
                #         quaternion = quaternion_from_euler(1.57-0.278+0.2,0.05-0.35+0.03,yaw-1)


                    # quaternion = quaternion_from_euler(1.57-0.278,0.05,1.57)
                # else:
                #     #quaternion = quaternion_from_euler(1.57-0.278+0.2,0.05-0.35,angle_aruco-1)
                #     quaternion = quaternion_from_euler(1.57-0.278+0.2,0.05-0.35+0.03,angle_aruco-1)
                    #print(angle_aruco,"chu")
                # print(quaternion)
                 #quaternion = quaternion_from_euler(0,1.57,angle_aruco)
                # quaternion_rot = quaternion_from_euler(0,1.57,angle_aruco)
                # quaternion = quaternion_multiply(quaternion_rot, quaternion)
                

            
            #   ->  Use center_aruco_list to get realsense depth and log them down. (divide by 1000 to convert mm to m)
                # distance_from_rgb = distance_from_rgb_list[i]
            #   ->  Use this formula to rectify x, y, z based on focal length, center value and size of image
                
                x = depth * (sizeCamX - int(center_aruco_list[i][0]) - centerCamX) / focalX
                y = depth * (sizeCamY - int(center_aruco_list[i][1]) - centerCamY) / focalY
                z = depth
                # print(x, y, z)
            #       where, 
            #               cX, and cY from 'center_aruco_list'
            #               distance_from_rgb is depth of object calculated in previous step
            #               sizeCamX, sizeCamY, centerCamX, centerCamY, focalX and focalY are defined above

            #   ->  Now, mark the center points on image frame using cX and cY variables with help of 'cv2.cirle' function 
                cv2.circle(self.cv_image, (int(center_aruco_list[i][0]), int(center_aruco_list[i][1])), 5, (0, 255, 0), -1)
            #   ->  Here, till now you receive coordinates from camera_link to aruco marker center position. 
            #       So, publish this transform w.r.t. camera_link using Geometry Message - TransformStamped 
            #       so that we will collect it's position w.r.t base_link in next step.
            # #       Use the following frame_id-
            #           frame_id = 'camera_link'
            #           child_frame_id = 'cam_<marker_id>'          Ex: cam_20, whe*re 20 is aruco marker ID
                
                
                if(center_aruco_list[i][0]>400 and center_aruco_list[i][0]<750):
                    quaternion = quaternion_from_euler(1.57-0.278,0.05,1.57)
                elif (center_aruco_list[i][0]>750):
                    # quaternion = quaternion_from_euler(1.57-0.278+0.2,0.05-0.35+0.03,yaw-1)
                    quaternion = quaternion_from_euler(1.57-0.278+0.2,0.05-0.35,0.11)
                else:
                    quaternion = quaternion_from_euler(0,1.57,1.57)

                self.publish_tf('camera_link', f'2022_cam_{id[0]}', (z, x,y), quaternion)
                # print (z,x,y)
            #     self.br.sendTransform(
            #         translation=(x, y, z),
            #         rotation=quaternion,
            #         time=self.clock.now().to_msg(),
            #         child_frame_id='cam_{}'.format(id),
            #         parent_frame_id='camera_link')

            # #   ->  Then finally lookup transform between base_link and obj frame to publish the TF
            # #       You may use 'lookup_transform' function to pose of obj frame w.r.t base_link 
                transform = TransformStamped()
                
                transform = self.listener.buffer.lookup_transform('base_link', '2022_cam_{}'.format(id[0]),tf2_ros.Time())
                # print('transform', transform.transform.translation)

            # # #   ->  And now publish TF between object frame and base_link
            # # #       Use the following frame_id-
            # # #           frame_id = 'base_link'
            # # #           child_frame_id = 'obj_<marker_id>'          Ex: obj_20, where 20 is aruco marker ID
                self.publish_tf('base_link', f'2022_obj_{id[0]}', (transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z), np.array([transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w]))
                
                # break
                # print("base se ",transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z)   
            #  self.br.sendTransform(
            #     translation=transform.translation,
            #     rotation=transform.rotation,
            #     time=self.clock.now().to_msg(),
            #     child_frame_id='obj_{}'.format(id),
            #     parent_frame_id='base_link')
                

            #   ->  At last show cv2 image window having detected markers drawn and center points located using 'cv2.imshow' function.
            #       Refer MD book on portal for sample image -> https://portal.e-yantra.org/
                cv2.imshow('RGB_frame', self.cv_image)
                cv2.waitKey(1)
                cv2.imshow('depth_frame', self.depth_image)
                cv2.waitKey(1)
                cv2.imwrite('CL#2022_color_image.png', self.cv_image)
            #   ->  NOTE:   The Z axis of TF should be pointing inside the box (Purpose of this will be known in task 1B)
            #               Also, auto eval script will be judging angular difference aswell. So, make sure that Z axis is inside the box (Refer sample images on Portal - MD book)

            ############################################
        except Exception as e:
            print(e, 'in process fxn')
            return None
        ############################################


##################### FUNCTION DEFINITION #######################

def main():
    '''
    Description:    Main function which creates a ROS node and spin around for the aruco_tf class to perform it's task
    '''

    rclpy.init(args=sys.argv)                                       # initialisation

    node = rclpy.create_node('aruco_tf_process')                    # creating ROS node

    node.get_logger().info('Node created: Aruco tf process')        # logging information

    aruco_tf_class = aruco_tf('aruco_tf_publisher')                                     # creating a new object for class 'aruco_tf'
    

    rclpy.spin(aruco_tf_class)                                      # spining on the object to make it alive in ROS 2 DDS

    aruco_tf_class.destroy_node()                                   # destroy node after spin ends

    rclpy.shutdown()                                                # shutdown process


if __name__ == '__main__':
    '''
    Description:    If the python interpreter is running that module (the source file) as the main program, 
                    it sets the special __name__ variable to have a value “__main__”. 
                    If this file is being imported from another module, __name__ will be set to the module’s name.
                    You can find more on this here -> https://www.geeksforgeeks.org/what-does-the-if-__name__-__main__-do/
    '''

    main()

     