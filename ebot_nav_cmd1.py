#! /usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from linkattacher_msgs.srv import AttachLink
from linkattacher_msgs.srv import DetachLink
from ebot_docking.srv import DockSw
import yaml
from example_interfaces.msg import String

"""
Basic navigation demo to go to pose.
"""
class Docking(Node):
     def __init__(self):
        super().__init__("docking_client")
        self.flag_timer = False
        self.navigator = BasicNavigator()
        self.publisher_= self.create_publisher(String,"car_update",10) ##publisher for car update!!
        self.car_update("False")
        # self.timer_=self.create_timer(0.5,self.car_update("False"))
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0.0
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.position.z = 0.05
        initial_pose.pose.orientation.z = 0.0
        initial_pose.pose.orientation.w = 1.0
        initial_pose.pose.orientation.y = 0.0
        initial_pose.pose.orientation.x = 0.0
        self.navigator.setInitialPose(initial_pose)
        self.navigator.waitUntilNav2Active()

        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.goal_pose.pose.position.x = self.coordinate()[1] + 0.13 #0.21
        self.goal_pose.pose.position.y = self.coordinate()[2] + 1.2
        self.goal_pose.pose.position.z = 0.00
        self.goal_pose.pose.orientation.z = 0.7068252
        self.goal_pose.pose.orientation.w = 0.7073883
        self.goal_pose.pose.orientation.y = 0.0
        self.goal_pose.pose.orientation.x = 0.0

        self.goal_pose_1 = PoseStamped()
        self.goal_pose_1.header.frame_id = 'map'
        self.goal_pose_1.header.stamp = self.navigator.get_clock().now().to_msg()
        self.goal_pose_1.pose.position.x = 0.029
        self.goal_pose_1.pose.position.y = -2.455
        self.goal_pose_1.pose.position.z = 0.00
        self.goal_pose_1.pose.orientation.z = 1.0
        self.goal_pose_1.pose.orientation.w = 0.0007963#-0.0142032 #0.0007963 #0.0007963 -0.0192025
        self.goal_pose_1.pose.orientation.y = 0.0
        self.goal_pose_1.pose.orientation.x = 0.0

        self.goal_pose_2 = PoseStamped()
        self.goal_pose_2.header.frame_id = 'map'
        self.goal_pose_2.header.stamp = self.navigator.get_clock().now().to_msg()
        self.goal_pose_2.pose.position.x = 0.89
        self.goal_pose_2.pose.position.y = -2.455
        self.goal_pose_2.pose.position.z = 0.00
        self.goal_pose_2.pose.orientation.z = 0.00
        self.goal_pose_2.pose.orientation.w = 1.0
        self.goal_pose_2.pose.orientation.y = 0.0
        self.goal_pose_2.pose.orientation.x = 0.0


        self.goal_pose_3 = PoseStamped()
        self.goal_pose_3.header.frame_id = 'map'
        self.goal_pose_3.header.stamp = self.navigator.get_clock().now().to_msg()
        self.goal_pose_3.pose.position.x = 0.0
        self.goal_pose_3.pose.position.y = 0.0
        self.goal_pose_3.pose.position.z = 0.00
        self.goal_pose_3.pose.orientation.z = 0.00
        self.goal_pose_3.pose.orientation.w = 1.0
        self.goal_pose_3.pose.orientation.y = 0.0
        self.goal_pose_3.pose.orientation.x = 0.0
        self.sent_request = False
        self.sent_request_home = False



        self.gripper_control_1 = self.create_client(AttachLink, '/ATTACH_LINK')
        self.gripper_control_2 = self.create_client(DetachLink, '/DETACH_LINK')
        self.docking_control = self.create_client(DockSw,'dock_control')
        while not self.gripper_control_1.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Link attacher service not available, waiting again...')
        while not self.gripper_control_2.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Link attacher service not available, waiting again...')
        while not self.docking_control.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('docking service not available, waiting again...')


     def send_request_dock(self):
         request1 = DockSw.Request()
         request1.linear_dock = True
         request1.orientation_dock = True
         request1.distance = 0.0
         request1.orientation = 1.65
         request1.rack_no = '1'
         self.future = self.docking_control.call_async(request1)
         while rclpy.ok():
            rclpy.spin_once(self)
            if self.future.done():
                try:
                    response = self.future.result()
                    self.sent_request = True
                except Exception as e:
                    self.get_logger().info(
                        f"Service call failed {e}"
                    )
                else:
                    self.get_logger().info(
                        f"Result of addition is {response.success}"
                        )
                    
                break
     def send_request_dock_to_home(self):
         request1 = DockSw.Request()
         request1.linear_dock = True
         request1.orientation_dock = True
         request1.distance = 0.75
         request1.orientation = 3.14
         request1.rack_no = '1'
         self.future = self.docking_control.call_async(request1)
         while rclpy.ok():
            rclpy.spin_once(self)
            if self.future.done():
                try:
                    response = self.future.result()
                    self.sent_request_home = True
                except Exception as e:
                    self.get_logger().info(
                        f"Service call failed {e}"
                    )
                else:
                    self.get_logger().info(
                        f"Result of addition is {response.success}"
                        )
                    
                break

     def attach_link(self):
        req1 = AttachLink.Request()
        req1.model1_name =  'ebot'     
        req1.link1_name  = 'ebot_base_link'       
        req1.model2_name = 'rack3'      
        req1.link2_name  = 'link'  
        self.gripper_control_1.call_async(req1)


     def detach_link(self):
        req = DetachLink.Request()
        req.model1_name =  'ebot'     
        req.link1_name  = 'ebot_base_link'       
        req.model2_name = f'rack{self.coordinate()[0]}'      
        req.link2_name  = 'link'  
        self.gripper_control_2.call_async(req)

     def gotopose(self , pose):
        if pose == self.goal_pose_1:
            self.reached = False
            print("chutiya")
        # else:

        self.navigator.goToPose(pose)
        i = 0
        while not self.navigator.isTaskComplete():
            
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated time of arrival: ' + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + ' seconds.')
                self.car_update("False")

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=800.0):
                    self.navigator.cancelTask()

                # Some navigation request change to demo preemption
                # if Duration.from_msg(feedback.navigation_time) > Duration(seconds=100.0):
                #     self.detach_link()

        
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')   
            self.reached = True    
            
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')
    
     def coordinate(self):
        with open('/home/dhananjay/config.yaml', 'r') as file:
            self.data = yaml.load(file)
        self.i = self.data['package_id'][0]
        self.x = self.data['position'][self.i-1]['rack3'][0]
        self.y = self.data['position'][self.i-1]['rack3'][1]
        self.yaw = self.data['position'][self.i-1]['rack3'][2]
        return [self.i,self.x,self.y,self.yaw]
     def car_update(self,val):
        msg=String()
        msg.data=val
        print(msg.data)
        self.publisher_.publish(msg) 
    
         
         
    
   



    
         


def main():
    rclpy.init()
    docking_client = Docking()
    docking_client.coordinate()
    docking_client.car_update("False")

    docking_client.gotopose(docking_client.goal_pose)
    docking_client.send_request_dock()
    docking_client.car_update("False")
    if docking_client.sent_request== True:
        docking_client.attach_link()
        docking_client.gotopose(docking_client.goal_pose_1)
        if (docking_client.reached):
           
            # docking_client.gotopose(docking_client.goal_pose_2)
            docking_client.send_request_dock_to_home()
            if docking_client.sent_request_home== True:
                docking_client.detach_link()
                print("bakland")
                docking_client.gotopose(docking_client.goal_pose_3)
                docking_client.car_update("True")
            
    else:
        print('future is false')
    docking_client.navigator.lifecycleShutdown()

    docking_client.destroy_node()
    rclpy.shutdown()
    exit(0)

    # Set our demo's initial pose
    # initial_pose = PoseStamped()
    # initial_pose.header.frame_id = 'map'
    # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    # initial_pose.pose.position.x = 0.0
    # initial_pose.pose.position.y = 0.0
    # initial_pose.pose.position.z = 0.05
    # initial_pose.pose.orientation.z = 0.0
    # initial_pose.pose.orientation.w = 1.0
    # initial_pose.pose.orientation.y = 0.0
    # initial_pose.pose.orientation.x = 0.0


    # navigator.setInitialPose(initial_pose)

    # # Activate navigation, if not autostarted. This should be called after setInitialPose()
    # # or this will initialize at the origin of the map and update the costmap with bogus readings.
    # # If autostart, you should `waitUntilNav2Active()` instead.
    # # navigator.lifecycleStartup()

    # # Wait for navigation to fully activate, since autostarting nav2
    # navigator.waitUntilNav2Active()

    # # If desired, you can change or load the map as well
    # # navigator.changeMap('/path/to/map.yaml')

    # # You may use the navigator to clear or obtain costmaps
    # # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # # global_costmap = navigator.getGlobalCostmap()
    # # local_costmap = navigator.getLocalCostmap()

    # # Go to our demos first goal pose
    # goal_pose = PoseStamped()
    # goal_pose.header.frame_id = 'map'
    # goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    # goal_pose.pose.position.x = 0.52
    # goal_pose.pose.position.y = 4.50
    # goal_pose.pose.position.z = 0.00
    # goal_pose.pose.orientation.z = 0.7068252
    # goal_pose.pose.orientation.w = 0.7073883
    # goal_pose.pose.orientation.y = 0.0
    # goal_pose.pose.orientation.x = 0.0



    # # sanity check a valid path exists  1.8, 1.5, 1.57
    # # path = navigator.getPath(initial_pose, goal_pose)

    # navigator.goToPose(goal_pose)

    # i = 0
    # while not navigator.isTaskComplete():
    #     ################################################
    #     #
    #     # Implement some code here for your application!
    #     #
    #     ################################################

    #     # Do something with the feedback
    #     i = i + 1
    #     feedback = navigator.getFeedback()
    #     if feedback and i % 5 == 0:
    #         print('Estimated time of arrival: ' + '{0:.0f}'.format(
    #               Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
    #               + ' seconds.')

    #         # Some navigation timeout to demo cancellation
    #         if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
    #             navigator.cancelTask()

    #         # Some navigation request change to demo preemption
    #         if Duration.from_msg(feedback.navigation_time) > Duration(seconds=1000.0):
    #             goal_pose.pose.position.x = -3.0
    #             navigator.goToPose(goal_pose)


    # result = navigator.getResult()
    # if result == TaskResult.SUCCEEDED:
    #     print('Goal succeeded!')
    #     docking_client.send_request_dock()
        
    # elif result == TaskResult.CANCELED:
    #     print('Goal was canceled!')
    # elif result == TaskResult.FAILED:
    #     print('Goal failed!')
    # else:
    #     print('Goal has an invalid return status!')
    # goal_pose_1 = PoseStamped()
    # goal_pose_1.header.frame_id = 'map'
    # goal_pose_1.header.stamp = navigator.get_clock().now().to_msg()
    # goal_pose_1.pose.position.x = 0.5
    # goal_pose_1.pose.position.y = -2.455
    # goal_pose_1.pose.position.z = 0.05
    # goal_pose_1.pose.orientation.z = 0.9999997
    # goal_pose_1.pose.orientation.w = 0.0007963
    # goal_pose_1.pose.orientation.y = 0.0
    # goal_pose_1.pose.orientation.x = 0.0


    # navigator.goToPose(goal_pose_1)

    # j = 0
    # while not navigator.isTaskComplete():
    #     ################################################
    #     #
    #     # Implement some code here for your application!
    #     #
    #     ################################################

    #     # Do something with the feedback
    #     j = j + 1
    #     feedback = navigator.getFeedback()
    #     if feedback and j % 5 == 0:
    #         print('Estimated time of arrival: ' + '{0:.0f}'.format(
    #               Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
    #               + ' seconds.')

    #         # Some navigation timeout to demo cancellation
    #         if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
    #             navigator.cancelTask()

    #         # Some navigation request change to demo preemption
    #         if Duration.from_msg(feedback.navigation_time) > Duration(seconds=80.0):
    #             goal_pose.pose.position.x = -3.0
    #             navigator.goToPose(goal_pose)

    # # Do something depending on the return code
    # result = navigator.getResult()
    # if result == TaskResult.SUCCEEDED:
    #     print('Goal succeeded!')
    #     docking_client.detach_link()
    # elif result == TaskResult.CANCELED:
    #     print('Goal was canceled!')
    # elif result == TaskResult.FAILED:
    #     print('Goal failed!')
    # else:
    #     print('Goal has an invalid return status!')
    
    # goal_pose_2 = PoseStamped()
    # goal_pose_2.header.frame_id = 'map'
    # goal_pose_2.header.stamp = navigator.get_clock().now().to_msg()
    # goal_pose_2.pose.position.x = 0.00
    # goal_pose_2.pose.position.y = 0.00
    # goal_pose_2.pose.position.z = 0.05
    # goal_pose_2.pose.orientation.z = 0.00
    # goal_pose_2.pose.orientation.w = 1.0
    # goal_pose_2.pose.orientation.y = 0.0
    # goal_pose_2.pose.orientation.x = 0.0

    # # rclpy.spin(docking_client)
    # navigator.goToPose(goal_pose_2)

    # k = 0
    # while not navigator.isTaskComplete():

    #     # Do something with the feedback
    #     k = k + 1
    #     feedback = navigator.getFeedback()
    #     if feedback and k % 5 == 0:
    #         print('Estimated time of arrival: ' + '{0:.0f}'.format(
    #               Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
    #               + ' seconds.')

    #         # Some navigation timeout to demo cancellation
    #         if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
    #             navigator.cancelTask()

    #         # Some navigation request change to demo preemption
    #         if Duration.from_msg(feedback.navigation_time) > Duration(seconds=80.0):
    #             goal_pose.pose.position.x = -3.0
    #             navigator.goToPose(goal_pose)


    # result = navigator.getResult()
    # if result == TaskResult.SUCCEEDED:
    #     print('Goal succeeded!')
    # elif result == TaskResult.CANCELED:
    #     print('Goal was canceled!')
    # elif result == TaskResult.FAILED:
    #     print('Goal failed!')
    # else:
    #     print('Goal has an invalid return status!')





   

    # docking_client.destroy_node()
    

    # goal_pose_1 = PoseStamped()
    # goal_pose_1.header.frame_id = 'map'
    # goal_pose_1.header.stamp = navigator.get_clock().now().to_msg()
    # goal_pose_1.pose.position.x = 2.0
    # goal_pose_1.pose.position.y = -7.0
    # goal_pose_1.pose.position.z = 0.05
    # goal_pose_1.pose.orientation.z = -0.7068252
    # goal_pose_1.pose.orientation.w = 0.70738830
    # goal_pose_1.pose.orientation.y = 0.0
    # goal_pose_1.pose.orientation.x = 0.0


    # navigator.goToPose(goal_pose_1)

    # j = 0
    # while not navigator.isTaskComplete():
    #     ################################################
    #     #
    #     # Implement some code here for your application!
    #     #
    #     ################################################

    #     # Do something with the feedback
    #     j = j + 1
    #     feedback = navigator.getFeedback()
    #     if feedback and j % 5 == 0:
    #         print('Estimated time of arrival: ' + '{0:.0f}'.format(
    #               Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
    #               + ' seconds.')

    #         # Some navigation timeout to demo cancellation
    #         if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
    #             navigator.cancelTask()

    #         # Some navigation request change to demo preemption
    #         if Duration.from_msg(feedback.navigation_time) > Duration(seconds=80.0):
    #             goal_pose.pose.position.x = -3.0
    #             navigator.goToPose(goal_pose)

    # # Do something depending on the return code
    # result = navigator.getResult()
    # if result == TaskResult.SUCCEEDED:
    #     print('Goal succeeded!')
    # elif result == TaskResult.CANCELED:
    #     print('Goal was canceled!')
    # elif result == TaskResult.FAILED:
    #     print('Goal failed!')
    # else:
    #     print('Goal has an invalid return status!')


    # goal_pose_2 = PoseStamped()
    # goal_pose_2.header.frame_id = 'map'
    # goal_pose_2.header.stamp = navigator.get_clock().now().to_msg()
    # goal_pose_2.pose.position.x = -3.0
    # goal_pose_2.pose.position.y = 2.5
    # goal_pose_2.pose.position.z = 0.05
    # goal_pose_2.pose.orientation.z = 0.706813
    # goal_pose_2.pose.orientation.w = 0.7073883
    # goal_pose_2.pose.orientation.y = 0.0
    # goal_pose_2.pose.orientation.x = 0.0

    # navigator.goToPose(goal_pose_2)

    # k = 0
    # while not navigator.isTaskComplete():
    #     ################################################
    #     #
    #     # Implement some code here for your application!
    #     #
    #     ################################################

    #     # Do something with the feedback
    #     k = k + 1
    #     feedback = navigator.getFeedback()
    #     if feedback and k % 5 == 0:
    #         print('Estimated time of arrival: ' + '{0:.0f}'.format(
    #               Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
    #               + ' seconds.')

    #         # Some navigation timeout to demo cancellation
    #         if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
    #             navigator.cancelTask()

    #         # Some navigation request change to demo preemption
    #         if Duration.from_msg(feedback.navigation_time) > Duration(seconds=80.0):
    #             goal_pose.pose.position.x = -3.0
    #             navigator.goToPose(goal_pose)


    # result = navigator.getResult()
    # if result == TaskResult.SUCCEEDED:
    #     print('Goal succeeded!')
    # elif result == TaskResult.CANCELED:
    #     print('Goal was canceled!')
    # elif result == TaskResult.FAILED:
    #     print('Goal failed!')
    # else:
    #     print('Goal has an invalid return status!')

    # # goal_pose_4 = PoseStamped()
    # # goal_pose_4.header.frame_id = 'map'
    # # goal_pose_4.header.stamp = navigator.get_clock().now().to_msg()
    # # goal_pose_4.pose.position.x = 0.0
    # # goal_pose_4.pose.position.y = 0.0
    # #goal_pose_4.pose.position.z = 0.05
    # # goal_pose_4.pose.orientation.z = 0.0
    # # goal_pose_4.pose.orientation.w = 1.0
    # # goal_pose_4.pose.orientation.y = 0.0
    # # goal_pose_4.pose.orientation.x = 0.0



    # # # sanity check a valid path exists  1.8, 1.5, 1.57
    # # # path = navigator.getPath(initial_pose, goal_pose)

    # # navigator.goToPose(goal_pose_4)

    # # s = 0
    # # while not navigator.isTaskComplete():
    # #     ################################################
    # #     #
    # #     # Implement some code here for your application!
    # #     #
    # #     ################################################

    # #     # Do something with the feedback
    # #     s = s + 1
    # #     feedback = navigator.getFeedback()
    # #     if feedback and s % 5 == 0:
    # #         print('Estimated time of arrival: ' + '{0:.0f}'.format(
    # #               Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
    # #               + ' seconds.')

    # #         # Some navigation timeout to demo cancellation
    # #         if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
    # #             navigator.cancelTask()

    # #         # Some navigation request change to demo preemption
    # #         if Duration.from_msg(feedback.navigation_time) > Duration(seconds=30.0):
    # #             goal_pose.pose.position.x = -3.0
    # #             navigator.goToPose(goal_pose)


    # # result = navigator.getResult()
    # # if result == TaskResult.SUCCEEDED:
    # #     print('Goal succeeded!')
    # # elif result == TaskResult.CANCELED:
    # #     print('Goal was canceled!')
    # # elif result == TaskResult.FAILED:
    # #     print('Goal failed!')
    # # else:
    # #     print('Goal has an invalid return status!')



    


if __name__ == '__main__':
    main()