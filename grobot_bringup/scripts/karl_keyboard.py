#!/usr/bin/env python3

import rospy
import sys
import select
import termios
import tty
from geometry_msgs.msg import Twist

msg = """
Control Your Robot!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity
space key, s : stop
CTRL-C to quit
"""

moveBindings = {
        'w': (1, 0),
        'x': (-1, 0),
        'a': (0, 1),
        'd': (0, -1),
        's': (0, 0),
        ' ': (0, 0)
}

speedBindings = {
        '1': (0.1, 0.1),
        '2': (0.2, 0.2),
        '3': (0.3, 0.3),
        '4': (0.4, 0.4),
        '5': (0.5, 0.5),
        '6': (0.6, 0.6),
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('keyboard_cmd_vel')
    pub = rospy.Publisher('/karl/cmd_vel', Twist, queue_size=10)

    speed = rospy.get_param("~speed", 0.2)
    turn = rospy.get_param("~turn", 0.2)
    
    x = 0
    th = 0
    status = 0

    try:
        print(msg)
        print(vels(speed, turn))
        while not rospy.is_shutdown():
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
            elif key in speedBindings.keys():
                speed = speedBindings[key][0]
                turn = speedBindings[key][1]

                print(vels(speed, turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                x = 0
                th = 0
                if (key == '\x03'):
                    break

            twist = Twist()
            twist.linear.x = x * speed * 5
            twist.angular.z = th * turn * 5
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
