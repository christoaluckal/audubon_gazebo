#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from gazebo_msgs.srv import GetModelState, GetModelStateRequest,SetModelState
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from gazebo_msgs.msg import ModelState

rospy.wait_for_service ('/gazebo/get_model_state')
rospy.wait_for_service('/gazebo/set_model_state')
get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

model = GetModelStateRequest()
model.model_name='calibration_plane'

def callback(data):
    global set_state,get_state,model
    orientation = get_state(model).pose.orientation
    loc = get_state(model).pose.position
    x,y,z,w = orientation.x,orientation.y,orientation.z,orientation.w
    r,p,y = (euler_from_quaternion([x,y,z,w]))

    yaw = y
    pitch = p
    roll = r

    if(abs(data.axes[0])>0.3):
        yaw = y-(0.03*data.axes[0])
    if(abs(data.axes[3])>0.3):
        pitch = p-(0.03*data.axes[3])
    if(abs(data.axes[4])>0.3):
        roll = r-(0.03*data.axes[4])

    quat = quaternion_from_euler(roll,pitch,yaw)
    state_msg = ModelState()
    state_msg.model_name = 'calibration_plane'
    state_msg.pose.position.x = loc.x
    state_msg.pose.position.y = loc.y
    state_msg.pose.position.z = loc.z
    state_msg.pose.orientation.x = quat[0]
    state_msg.pose.orientation.y = quat[1]
    state_msg.pose.orientation.z = quat[2]
    state_msg.pose.orientation.w = quat[3]
    set_state( state_msg )


def talker():
    rospy.init_node('calibration_node', anonymous=True)
    rospy.Subscriber("/joy", Joy, callback)
    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        # result = get_state(model)
        # # print(result)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        print("ERR")
        pass