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
        print("pose : ", marker_translation)

def follow_marker():
    global marker_id, marker_translation,marker_rotation

    rospy.init_node('follower_turtle', anonymous=True)
    rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, fiducial_callback)
    turtlebot_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(10)  # 10Hz

    while not rospy.is_shutdown():
        if marker_id is not None:
            cmd_vel_msg = Twist()
            # Set linear and angular velocity based on marker position
            cmd_vel_msg.linear.x = 0.1  # Set linear velocity
            cmd_vel_msg.angular.z = -0.1 * marker_translation.x  # Set angular velocity based on x translation
            turtlebot_vel_pub.publish(cmd_vel_msg)
        else:
            # Stop if marker is not detected
            cmd_vel_msg = Twist()
            turtlebot_vel_pub.publish(cmd_vel_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        follow_marker()
    except rospy.ROSInterruptException:
        pass