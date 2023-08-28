#!/usr/bin/env python3

import rospy
from fiducial_msgs.msg import FiducialTransformArray
from geometry_msgs.msg import Twist
from math import atan2, sqrt, degrees
from tf.transformations import euler_from_quaternion

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

        print("translation : ",marker_translation.x)





def follow_marker():
    global marker_id, marker_translation, marker_rotation

    rospy.init_node('follower_turtle', anonymous=True)
    rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, fiducial_callback)
    turtlebot_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    try:
        follow_marker()
    except rospy.ROSInterruptException:
        pass
