from __future__ import print_function

import sys

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


moveBindings = {
    'w': (1  ,0),
    'a': (0 ,1),
    'x': (-1 ,0),
    'd': (0  ,-1),
    'q': (1 ,1),
    'e': (1  ,-1),
    'z': (-1 ,1),
    'c': (-1 ,-1),
    's': (0 , 0),
}

stepBindings = {
    'i': (0.02, 0),
    ',': (-0.02, 0),
    'o': (0, 0.1),
    '.': (0, -0.1),
    'k': (0.05, 0),
    'l': (0, 0.5),
}

msg = """

-----------------------------------
========== Move Around! ===========
-----------------------------------
    q   w   e           i  o
    a   s   d
    z   x   c           <  >

w/x : increase/decrease linear velocity (max value : 0.2362)
a/d : increase/decrease angular velocity (max value : +- 0.5233)

i/< : increase/decrease linear step size (default : 0.02)
o/> : increase/decrease angular step size (default : 0.1)

ctl + c : quit
"""

e = """
Connection Failed
"""
def getKey(settings):
    if sys.platform =='win32':
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(speed, turn):
    return 'currently:\tspeed %s\tturn %s ' %(speed, turn)

def constrain(input, low, high):
    if input < low:
        input = low
    elif input > high:
        input = high
    else:
        input = input
    return input

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node('dummy1_teleop_twist')
    pub = node.create_publisher(Twist, 'cmd_vel', 10)

    speed = 0.05
    turn = 0.5
    target_linear_vel = 0.0
    target_angular_vel = 0.0
    status = 0.0
    max_linear_vel = 0.2362
    max_angular_vel = 0.5233

    try:
        print(msg)
        print(vels(speed,turn))
        while True:
            keys = getKey(settings)
            if keys in moveBindings.keys():
                target_linear_vel = moveBindings[keys][0]
                target_angular_vel = moveBindings[keys][1]
            elif keys in stepBindings.keys():
                speed = constrain(speed + stepBindings[keys][0], - max_linear_vel ,  max_linear_vel)
                turn = constrain(turn + stepBindings[keys][1], -max_angular_vel, max_angular_vel)
                if keys == 'k':
                    speed = 0.05
                if keys == 'l':
                    turn = 0.5
                print( vels( speed, turn))
                if (status == 10):
                    print(msg)
                status = (status + 1) % 11
            else:
                target_linear_vel = 0.0
                target_angular_vel = 0.0
                if (keys == '\x03'):
                    break

            twist = Twist()
            twist.linear.x =  target_linear_vel *  speed
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z =  target_angular_vel *  turn
            pub.publish(twist)
    except Exception as err:
        print(e)
        print(err)

    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

        restoreTerminalSettings(settings)


if __name__=='__main__':
    main()
