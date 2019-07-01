#! /usr/bin/env python

import rospy
import math
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

"""
    Notes :
    Code is functionnal but quite a few improvements can be made :

    - Pepper stops all the time when in autonomous life mode and when security distances are activated.
    Need to make a launch file that deactivates stuff on demand. Learn to use ROS dynamic parameters.
    Use a controller button to activate or deactivate these modes.
    - Use arrows to change max speeds (up down linear and left right angular).
    - Add button to shut down or at least put to sleep.
    - Add debugging buttons that make Pepper say whatever state it is in.

    - Add head control through arrows.
    - Add button to activate lights.
    - Add button to start interaction (Make a list of sentences, and have it pick them at random till expiration then
    replenish sentence list).
    - Add button control to start control of left arm. Same for right arm.
    - Add button for knock door action
"""


class Teleop:

    def __init__(self, debug, joy_topic, cmd_vel_topic, speech_topic, max_linear_speed, max_angular_speed):
        self.debug = debug
        
        self.btns = {"triangle": 0, "circle": 1, "cross": 2, "square": 3,
                     "l1": 4, "r1": 5, "l2": 6, "r2": 7,
                     "select": 8, "start": 9,
                     "press_l_joy": 10, "press_r_joy": 11}

        # Tuple always corresponds to (left/right, up/down)
        # Last value in joy.axes array (at 6) is extra
        self.axes = {"l_joy": (0, 1), "r_joy": (2, 3), "arrows": (4, 5)}

        self.joy_state_subscriber = rospy.Subscriber(joy_topic, Joy, self.cur_joy_callback)
        self.cmd_vel_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
        self.speech_publisher = rospy.Publisher(speech_topic, String, queue_size=1)

        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed

        self.last_joy = Joy()
        self.last_joy.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.last_joy.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.twist_to_apply = Twist()

    def print_readable_joy(self, joy):
        for name, id in self.btns.items():
            if joy.buttons[id] != 0:
                print(name + " button is pressed")

        for name in self.axes.keys():
            norm, angle = self.get_joystick_norm_angle(joy, name)
            if norm != 0.0:
                print(name + " is at " + str(math.degrees(angle)) + " degrees and norm at " + str(norm))

    def get_joystick_norm_angle(self, joy, joy_name):
        # Note : 1.0 is up, -1.0 is down, 1.0 is left and -1.0 is right
        x = -joy.axes[self.axes[joy_name][0]]
        y = joy.axes[self.axes[joy_name][1]]
        joystick_angle_radians = math.atan2(x, y)
        # Funny math to go from a "square" coordinate system to a "circle" one
        # See http://mathproofs.blogspot.com/2005/07/mapping-square-to-circle.html
        # Found from https://stackoverflow.com/questions/1621831/how-can-i-convert-coordinates-on-a-square-to-coordinates-on-a-circle
        joystick_norm = math.sqrt(math.pow(x, 2) + math.pow(y, 2) - math.pow(x * y, 2))
        return joystick_norm, joystick_angle_radians

    def cur_joy_callback(self, cur_joy):
        if self.debug:
            self.print_readable_joy(cur_joy)

        twist_to_apply = Twist()

        cur_l_joy_norm, cur_l_joy_angle = self.get_joystick_norm_angle(cur_joy, "l_joy")
        cur_r_joy_norm, cur_r_joy_angle = self.get_joystick_norm_angle(cur_joy, "r_joy")
        cur_arrows_norm, cur_arrows_angle = self.get_joystick_norm_angle(cur_joy, "arrows")

        if cur_r_joy_norm != 0.0:
            twist_to_apply.linear.x = cur_r_joy_norm * math.cos(cur_r_joy_angle) * self.max_linear_speed
            twist_to_apply.linear.y = cur_r_joy_norm * math.sin(-cur_r_joy_angle) * self.max_linear_speed

        if cur_l_joy_norm != 0.0:
            twist_to_apply.angular.z = cur_l_joy_norm * (1.0 if cur_l_joy_angle < 0.0 else -1.0) * self.max_angular_speed

        is_cross_pressed = cur_joy.buttons[self.btns["cross"]]

        if is_cross_pressed:
            self.publish_msg("Bonjour !")

        is_circle_pressed = cur_joy.buttons[self.btns["circle"]]

        if is_circle_pressed:
            self.publish_msg("Venez parler avec nos etudiants de TC")

        is_arrow_up = cur_joy.buttons[self.axes["arrows"][0]]

        if self.debug:
            rospy.loginfo("twist_to_apply : " + str(twist_to_apply))

        self.twist_to_apply = twist_to_apply

        self.last_joy = cur_joy

    def publish_vel(self):
        self.cmd_vel_publisher.publish(self.twist_to_apply)

    def publish_msg(self, msg):
        self.speech_publisher.publish(msg)

if __name__ == '__main__':
    rospy.init_node('teleoperator', anonymous=True)
    teleop = Teleop(debug=True,
                    joy_topic="/joy",
                    cmd_vel_topic="/cmd_vel",
                    speech_topic="/speech",
                    max_linear_speed=0.4,
                    max_angular_speed=5.0)

    rate = rospy.Rate(30) # 30Hz
    while not rospy.is_shutdown():
        teleop.publish_vel()
        rate.sleep()
