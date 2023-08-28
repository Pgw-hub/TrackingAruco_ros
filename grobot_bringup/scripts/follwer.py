#!/usr/bin/env python3

import rospy
from fiducial_msgs.msg import FiducialTransformArray
from geometry_msgs.msg import Twist

# Global variables to store marker information
marker_id = None
marker_translation = None
marker_rotation = None

def fiducial_callback(data):
    global marker_id, marker_translation, marker_rotation
    if data.transforms:
        marker_id = data.transforms[0].fiducial_id
        marker_translation = data.transforms[0].transform.translation
        marker_rotation = data.transforms[0].transform.rotation
        print("pose : ", marker_translation.x)

def follow_marker():
    global marker_id, marker_translation,marker_rotation

    rospy.init_node('follower_turtle', anonymous=True)
    rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, fiducial_callback)
    turtlebot_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(10)  # 10Hz

    while not rospy.is_shutdown():
        cmd_vel_msg = Twist()
        #마커를 찾을시 조향각, 속도 조절.
        if marker_id is not None:
            #longitudinal
            if marker_translation.z < 0.9 :
                cmd_vel_msg.linear.x = 0.0
            else :
                cmd_vel_msg.linear.x = 0.2
            
            #lateral
            if marker_translation.x > 0 : #로봇기준 왼쪽
                cmd_vel_msg.angular.z = -0.1
            else :
                cmd_vel_msg.angular.z = 0.1 
            turtlebot_vel_pub.publish(cmd_vel_msg)
        #마커를 못찾을시 정지
        else :
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.angular.z = 0.1
            turtlebot_vel_pub.publish(cmd_vel_msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        follow_marker()
    except rospy.ROSInterruptException:
        pass
