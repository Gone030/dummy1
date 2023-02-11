from __future__ import print_function

import rclpy
from geometry_msgs.msg import Twist
from pynput.keyboard import Key, Listener
from rclpy.node import Node
from rclpy.qos import QoSProfile

import os
import select
import sys

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

class teleop(Node):
    def __init__(self):
        super().__init__('dummy1_teleop_twist_keyboard')
        qos_profile = QoSProfile(depth = 10)

        self.settings = self.saveTerminalSettings()

        self.keys = set()
        self.status = 0
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0
        self.pub = self.create_publisher(Twist, 'cmd_vel', qos_profile)

        self.max_linear_vel = 0.2362
        self.max_angular_vel = 0.5233

        self.linear_step_size = 0.02
        self.angular_step_size = 0.1

        self.speed = 0.05
        self.turn  = 0.5

        print(msg)
        print(self.vels(self.speed, self.turn))
        with Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )as listener:
            listener.join()
    def vels(self, speed, turn):
        return 'currently:\tspeed %s\tturn %s ' %(speed, turn)

    def constrain(self, input, low, high):
        if input < low:
            input = low
        elif input > high:
            input = high
        else:
            input = input
        return input

    def on_press(self, key):
        tty.setraw(sys.stdin.fileno())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

        if key == Key.esc:
            return False

        if hasattr(key, 'char'):
            key = key.char
            self.keys.add(key)
            self.move()

    def on_release(self, key):
        tty.setraw(sys.stdin.fileno())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        if hasattr(key, 'char'):
            key = key.char
            if key in self.keys:
                self.keys.remove(key)
                self.move()
    def saveTerminalSettings(self):
        if sys.platform == 'win32':
            return None
        return termios.tcgetattr(sys.stdin)

    def restoreTerminalSettings(self, old_settings):
        if sys.platform == 'win32':
            return
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def move(self):
        try:
            keys = ''.join(self.keys)
            if keys in moveBindings.keys():
                self.target_linear_vel = moveBindings[keys][0]
                self.target_angular_vel = moveBindings[keys][1]
            elif keys in stepBindings.keys():
                self.speed = self.constrain(self.speed + stepBindings[keys][0], - self.max_linear_vel , self.max_linear_vel)
                self.turn = self.constrain(self.turn + stepBindings[keys][1], -self.max_angular_vel, self.max_angular_vel)
                if keys == 'k':
                    self.speed = 0.05
                if keys == 'l':
                    self.turn = 0.5
                print(self.vels(self.speed, self.turn))
                if (self.status == 10):
                    print(msg)
                self.status = (self.status + 1) % 11
            else:
                self.target_linear_vel = 0.0
                self.target_angular_vel = 0.0

            twist = Twist()
            twist.linear.x = self.target_linear_vel * self.linear_step_size
            twist.angular.z = self.target_angular_vel * self.angular_step_size
            self.pub.publish(twist)
        except Exception as err:
            print(e)
            print(err)

            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args = None):
    rclpy.init(args=args)
    node = teleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        node.pub.publish(twist)
        node.destroy_node()
        rclpy.shutdown()
if __name__=='__main__':
    main()
