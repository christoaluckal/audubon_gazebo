#! /usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
from std_srvs.srv import SetBool, SetBoolResponse
from ackermann_msgs.msg import AckermannDrive

def handle_reset():
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    clear_path = rospy.ServiceProxy('/car_1/clear_path', SetBool)
    drive_msg =AckermannDrive()

    clear_input_pub = rospy.Publisher('/car_1/command', AckermannDrive, queue_size=1)
    clear_input_pub.publish(drive_msg)

    try:
        state_msg = ModelState()
        state_msg.model_name = 'car_1'
        state_msg.pose.position.x =  0
        state_msg.pose.position.y = 0
        state_msg.pose.position.z =  0
        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 0
        state_msg.pose.orientation.w = 1

        state_msg.twist.linear.x = 0
        state_msg.twist.linear.y = 0
        state_msg.twist.linear.z = 0

        state_msg.twist.angular.x = 0
        state_msg.twist.angular.y = 0
        state_msg.twist.angular.z = 0
        gazebo_resp = set_state(state_msg) 
    except rospy.ServiceException as e:
        return SetBoolResponse(False, str(e))

    rospy.sleep(1)

rospy.init_node('reset_car_node')

# Wait for reset services to spawn
rospy.wait_for_service('/gazebo/set_model_state')
rospy.wait_for_service('/car_1/clear_path')

handle_reset()
rospy.spin()

