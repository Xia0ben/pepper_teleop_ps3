#! /usr/bin/env python

import rospy
import math
import time
import datetime
import pickle
import os

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from pepper_pose_for_nav.srv import MoveHeadAtPosition
from dialogue_hri_srvs.srv import TakePicture

from rospy.exceptions import ROSException, ROSInterruptException

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

    def __init__(self, debug, joy_topic, cmd_vel_topic, speech_topic, max_linear_speed, max_angular_speed, picture_folder, data_joystick_folder, mode):
        self.debug = debug

        self.btns_conf_file = "{0}/btns.conf".format(data_joystick_folder)
        if os.path.isfile(self.btns_conf_file):
            with open(self.btns_conf_file, "rb") as btns_conf:
                self.btns = pickle.load(btns_conf)
        else:
            self.btns = {"triangle": 0, "circle": 1, "cross": 2, "square": 3,
                         "l1": 4, "r1": 5, "l2": 6, "r2": 7,
                         "select": 8, "start": 9,
                         "press_l_joy": 10, "press_r_joy": 11}

        self.btns_configuration_list = sorted(self.btns.keys())
        self.btns_configuration_curr = ""
        self.prev_btn_pressed = -1

        # Tuple always corresponds to (left/right, up/down)
        # Last value in joy.axes array (at 6) is extra
        self.axes_conf_file = "{0}/axes.conf".format(data_joystick_folder)
        if os.path.isfile(self.axes_conf_file):
            with open(self.axes_conf_file, "rb") as axes_conf:
                self.axes = pickle.load(axes_conf)
        else:
            self.axes = {"l_joy": [0, 1], "r_joy": [2, 3], "arrows": [4, 5]}

        self.axes_configuration_list = sorted(self.axes.keys())
        self.axes_configuration_curr = ""
        self.axes_configuration_dir = 0
        self.prev_axe_chosen = -1

        self.mode = mode

        self.joy_state_subscriber = rospy.Subscriber(joy_topic, Joy, self.cur_joy_callback)
        self.cmd_vel_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
        self.speech_publisher = rospy.Publisher(speech_topic, String, queue_size=1)

        self.move_head_srv = rospy.ServiceProxy('move_head_pose_srv', MoveHeadAtPosition)
        self.take_pictures_srv = rospy.ServiceProxy('take_picture_service', TakePicture)
        self.ok_move_head_srv = False
        self.ok_take_pictures_srv = False
        # try:
        #     self.ok_move_head_srv = rospy.wait_for_service('move_head_pose_srv', timeout=10.0)
        #     self.ok_take_pictures_srv = rospy.wait_for_service('take_picture_service', timeout=10.0)
        # except (ROSException, ROSInterruptException) as e:
        #     rospy.logwarn("Unable to connect service - {0}".format(e))

        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.picture_folder = picture_folder

        self.last_joy = Joy()
        self.last_joy.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.last_joy.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.curr_head_pitch = 0.0
        self.curr_head_yaw = 0.0
        self.head_track = True #Enable head tracking. Helps to keep head in a given position.
        if (self.ok_move_head_srv == True):
            self.move_head_srv(self.curr_head_pitch, self.curr_head_yaw, self.head_track)
        self.head_pitch_step = 0.05
        self.head_yaw_step = 0.05
        self.head_pitch_lim = 0.5
        self.head_yaw_lim = 0.5

        self.twist_to_apply = Twist()

        if self.mode == 0:
            print "Buttons Configuration. Press any button to start. "
        elif self.mode == 1:
            print "Axes Configuration. Press any button to start"
        elif self.mode == 2:
            print "No configuration needed. Start to play."
        else:
            print "Value mode = {0} unsupported"

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
        # If normal use
        if self.mode == 0:
            if self.btns_configuration_curr == "":
                if len(self.btns_configuration_list) > 0:
                    self.btns_configuration_curr = self.btns_configuration_list.pop(0)
                    print "Press {0}".format(self.btns_configuration_curr)
                else:
                    print "Buttons Configuration done."
                    with open(self.btns_conf_file, "wb") as btns_conf:
                        pickle.dump(self.btns, btns_conf)
                        btns_conf.close()
                    self.mode = 1
                    print self.btns
                    print "Axes Configuration. Press any button to start"
            else:
                for i in range(12):
                    if cur_joy.buttons[i] and (i != self.prev_btn_pressed):
                        self.btns[self.btns_configuration_curr] = i
                        self.btns_configuration_curr = ""
                        self.prev_btn_pressed = i
                        break
        elif self.mode == 1:
            if self.axes_configuration_curr == "":
                if len(self.axes_configuration_list) > 0:
                    self.axes_configuration_curr = self.axes_configuration_list.pop(0)
                    print "Press {0} Left / Right".format(self.axes_configuration_curr)
                else:
                    print "Axes Configuration done."
                    print self.axes
                    with open(self.axes_conf_file, "wb") as axes_conf:
                        pickle.dump(self.axes, axes_conf)
                        axes_conf.close()
                    self.mode = 2
            else:
                max = 0.5
                i_max = -1
                for i in range(6):
                    if abs(cur_joy.axes[i]) > max:
                        i_max = i
                        max = abs(cur_joy.axes[i])
                if (i_max >= 0) and (i_max != self.prev_axe_chosen):
                    self.axes[self.axes_configuration_curr][self.axes_configuration_dir] = i_max
                    self.prev_axe_chosen = i_max
                    if self.axes_configuration_dir == 1:
                        self.axes_configuration_curr = ""
                        self.axes_configuration_dir = 0
                    else:
                        self.axes_configuration_dir = 1
                        print "Press {0} Up / Down".format(self.axes_configuration_curr)
        elif self.mode == 2:
            # Debug prints
            if self.debug:
                self.print_readable_joy(cur_joy)
            # Get directionnal inputs
            cur_l_joy_norm, cur_l_joy_angle = self.get_joystick_norm_angle(cur_joy, "l_joy")
            cur_r_joy_norm, cur_r_joy_angle = self.get_joystick_norm_angle(cur_joy, "r_joy")
            cur_arrows_norm, cur_arrows_angle = self.get_joystick_norm_angle(cur_joy, "arrows")
            # Compute twist
            twist_to_apply = Twist()
            if cur_r_joy_norm != 0.0:
                twist_to_apply.linear.x = cur_r_joy_norm * math.cos(cur_r_joy_angle) * self.max_linear_speed
                twist_to_apply.linear.y = cur_r_joy_norm * math.sin(-cur_r_joy_angle) * self.max_linear_speed
            if cur_l_joy_norm != 0.0:
                twist_to_apply.angular.z = cur_l_joy_norm * (1.0 if cur_l_joy_angle < 0.0 else -1.0) * self.max_angular_speed
            # Compute head movements
            if cur_arrows_norm != 0.0:
                #Pitch - Upper arrow -> head up
                head_pitch_to_incr = - cur_arrows_norm * math.cos(cur_arrows_angle) * self.head_pitch_step
                head_pitch_to_apply = self.curr_head_pitch + head_pitch_to_incr
                head_pitch_to_apply = max(-self.head_pitch_lim, min(self.head_pitch_lim, head_pitch_to_apply))
                #Yaw - Left arrow -> turn to the left
                head_yaw_to_incr = cur_r_joy_norm * math.sin(-cur_r_joy_angle) * self.head_yaw_step
                head_yaw_to_apply = self.curr_head_yaw + head_yaw_to_incr
                head_yaw_to_apply = max(-self.head_yaw_lim, min(self.head_yaw_lim, head_yaw_to_apply))
                #Move head
                # self.move_head_srv(head_pitch_to_apply, head_yaw_to_apply, self.head_track)
                print head_pitch_to_apply, head_yaw_to_apply
                #save values
                self.curr_head_pitch = head_pitch_to_apply
                self.curr_head_yaw = head_yaw_to_apply
            # Come one... say hello
            is_cross_pressed = cur_joy.buttons[self.btns["cross"]]
            if is_cross_pressed:
                self.publish_msg("Bonjour !")
            # Go play with your new friends
            is_circle_pressed = cur_joy.buttons[self.btns["circle"]]
            if is_circle_pressed:
                self.publish_msg("Venez parler avec nos etudiants de TC")
            # Take a picture (here we go Instagram...)
            is_square_pressed = cur_joy.buttons[self.btns["square"]]
            if is_square_pressed:
                timestamp = datetime.datetime.fromtimestamp(time.time())
                filename = "{0}.png".format(timestamp.isoformat('_'))
                print "ChEeeeeeSe gne"
                # self.take_pictures_srv("{0}/{1}".format(self.picture_folder, filename)
            # No purpose
            is_arrow_up = cur_joy.buttons[self.axes["arrows"][0]]
            # Debug prints
            if self.debug:
                rospy.loginfo("twist_to_apply : " + str(twist_to_apply))
            # Save values
            self.twist_to_apply = twist_to_apply
            self.last_joy = cur_joy
        else:
            pass

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
                    max_angular_speed=5.0,
                    picture_folder="/home/astro/catkin_robocup2019/data/pictures",
                    data_joystick_folder="/home/astro/catkin_robocup2019/data/pepper3_teleop_ps3",
                    mode=2)


    rate = rospy.Rate(30) # 30Hz
    while not rospy.is_shutdown():
        teleop.publish_vel()
        rate.sleep()
