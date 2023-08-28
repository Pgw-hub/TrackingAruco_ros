#!/usr/bin/env python3

import rospy
import math
from fiducial_msgs.msg import FiducialTransformArray
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, error):
        self.integral += error
        derivative = error - self.prev_error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

class FollowerNode:
    def __init__(self):
        rospy.init_node('follower_node', anonymous=True)
        self.sub = rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, self.fiducial_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pid_translation = PIDController(kp=1.0, ki=0.0, kd=0.0)
        self.pid_rotation = PIDController(kp=1.0, ki=0.0, kd=0.0)
        self.reference_translation = [-0.061, -0.216, 0.812]
        self.reference_rotation = [0.009, 0.999, -0.008, 0.017]
        self.target_distance_threshold = 0.01  # Tolerance for translation
        self.target_rotation_threshold = 0.1  # Tolerance for rotation

    def fiducial_callback(self, data):
        if data.transforms:
            translation = data.transforms[0].transform.translation
            rotation = data.transforms[0].transform.rotation


            translation_error = math.sqrt(
                (self.reference_translation[0] - translation.x) ** 2 +
                (self.reference_translation[1] - translation.y) ** 2 +
                (self.reference_translation[2] - translation.z) ** 2
            )

            _, _, yaw = euler_from_quaternion([rotation.x, rotation.y, rotation.z, rotation.w])
            rotation_error = abs(yaw)

            cmd_vel_msg = Twist()
            #if z_distance is close, stop.
            if abs(self.reference_translation[2] - translation.z) < 0.1 :
                print("I can stop!!")
                cmd_vel_msg.linear.x = 0.0
                cmd_vel_msg.angular.z = 0.0
            else :
                cmd_vel_msg.linear.x = self.pid_translation.compute(translation_error)
            cmd_vel_msg.angular.z = self.pid_rotation.compute(rotation_error)
            

            # if translation_error < self.target_distance_threshold and rotation_error < self.target_rotation_threshold:
            #     cmd_vel_msg.linear.x = 0.0
            #     cmd_vel_msg.angular.z = 0.0

            self.cmd_vel_pub.publish(cmd_vel_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        follower_node = FollowerNode()
        follower_node.run()
    except rospy.ROSInterruptException:
        pass
