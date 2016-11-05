#!/usr/bin/env python

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Baxter RSDK Joint Position Example: keyboard
"""
import argparse

import rospy
import sys

import baxter_interface
import baxter_external_devices
from baxter_interface import gripper as baxter_gripper
import time

from baxter_interface import CHECK_VERSION

joint_state1 = [0.0, -0.0897378760912967, 3.0537722534828875, 0.9898011033830633, -0.6051554208207958, 
1.0473253829287663, 1.6505633277647052, 1.8104808249017597, 1.609145846491799, -2.9855101084219866,
1.9481556006144753, 0.7673738891396782, 0.7485826244880819, 1.4825924314912524, -1.5619759372643225, -0.32597091742565043, -12.565987119160338]

joint_state2 = [0.0, -0.08935438089432535, 3.0533887582859163, 2.182087670767001, -0.40803888957752005, 
0.3175340230922806, 1.1408982109897765, 1.517106999218674, 0.9510680884889565, -2.934888742421768, 1.940102201478077, 
0.7090826192000325, 0.7401457301547121, 1.4749225275518254, -1.549320595764268, -0.3255874222286791, -12.565987119160338]

joint_states = {'initial': joint_state1, 'next':joint_state2}



def map_keyboard():
    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    def set_j(limb, joint_name, delta):
        current_position = limb.joint_angle(joint_name)
        joint_command = {joint_name: current_position + delta}
        limb.set_joint_positions(joint_command)

    bindings = {
    #   key: (function, args, description)
        '9': (set_j, [left, lj[0], 0.1], "left_s0 increase"),
        '6': (set_j, [left, lj[0], -0.1], "left_s0 decrease"),
        '8': (set_j, [left, lj[1], 0.1], "left_s1 increase"),
        '7': (set_j, [left, lj[1], -0.1], "left_s1 decrease"),
        'o': (set_j, [left, lj[2], 0.1], "left_e0 increase"),
        'y': (set_j, [left, lj[2], -0.1], "left_e0 decrease"),
        'i': (set_j, [left, lj[3], 0.1], "left_e1 increase"),
        'u': (set_j, [left, lj[3], -0.1], "left_e1 decrease"),
        'l': (set_j, [left, lj[4], 0.1], "left_w0 increase"),
        'h': (set_j, [left, lj[4], -0.1], "left_w0 decrease"),
        'k': (set_j, [left, lj[5], 0.1], "left_w1 increase"),
        'j': (set_j, [left, lj[5], -0.1], "left_w1 decrease"),
        '.': (set_j, [left, lj[6], 0.1], "left_w2 increase"),
        'n': (set_j, [left, lj[6], -0.1], "left_w2 decrease"),
        ',': (grip_left.close, [], "left: gripper close"),
        'm': (grip_left.open, [], "left: gripper open"),
        '/': (grip_left.calibrate, [], "left: gripper calibrate"),

        '4': (set_j, [right, rj[0], 0.1], "right_s0 increase"),
        '1': (set_j, [right, rj[0], -0.1], "right_s0 decrease"),
        '3': (set_j, [right, rj[1], 0.1], "right_s1 increase"),
        '2': (set_j, [right, rj[1], -0.1], "right_s1 decrease"),
        'r': (set_j, [right, rj[2], 0.1], "right_e0 increase"),
        'q': (set_j, [right, rj[2], -0.1], "right_e0 decrease"),
        'e': (set_j, [right, rj[3], 0.1], "right_e1 increase"),
        'w': (set_j, [right, rj[3], -0.1], "right_e1 decrease"),
        'f': (set_j, [right, rj[4], 0.1], "right_w0 increase"),
        'a': (set_j, [right, rj[4], -0.1], "right_w0 decrease"),
        'd': (set_j, [right, rj[5], 0.1], "right_w1 increase"),
        's': (set_j, [right, rj[5], -0.1], "right_w1 decrease"),
        'v': (set_j, [right, rj[6], 0.1], "right_w2 increase"),
        'z': (set_j, [right, rj[6], -0.1], "right_w2 decrease"),
        'c': (grip_right.close, [], "right: gripper close"),
        'x': (grip_right.open, [], "right: gripper open"),
        'b': (grip_right.calibrate, [], "right: gripper calibrate"),
     }
    done = False
    print("Controlling joints. Press ? for help, Esc to quit.")
    while not done and not rospy.is_shutdown():
        side = raw_input('Enter left or right')
        
        if side == 'left':
            name_list = lj
        else:
            name_list = rj
        # for i in range(7):
        #     print 'joint name: ', name_list[i]

        side = baxter_interface.Limb(side)
        new_angles = []
        delta = lambda new_angle, curr_angle: new_angle-curr_angle

        flag = raw_input('Do you want to set up constant position: ')
        if flag == '1':
            command = 'initial'
        else:    
            for n in range(7):
                new_angle = raw_input('Enter an angle for joint' + name_list[n] + ': ')
                new_angle = float(new_angle)
                new_angles.append(new_angle)

        move(command, side, name_list, delta, set_j)
        rospy.sleep(1.0)

        gripper = baxter_gripper.Gripper('left')
        gripper.close(block=True)
        rospy.sleep(1.0)
        print('CLOSE Done!\n')      

        command = 'next' 
        move(command, side, name_list, delta, set_j)

def move(command, side, name_list, delta, set_j):
    tol = 0.01
    new_angles =[joint_states[command][4], joint_states[command][5], joint_states[command][2], joint_states[command][3], joint_states[command][6], joint_states[command][7], joint_states[command][8]]
    for n in range(7):
            while abs(new_angles[6-n] - side.joint_angle(name_list[6-n])) > tol:
                print "NEW ANGLE: ", str(new_angles[6-n]), "Current Angle:", str(side.joint_angle(name_list[6-n]))
                # print (name_list[n] +':' + str(side.joint_angle(name_list[n])))
                change = delta(new_angles[6-n], side.joint_angle(name_list[6-n]))
                print "Not there yet, moving by ", change
                set_j( side, name_list[6-n] , change)
                time.sleep(0.1)
    print("Commands complete")

def main():
    """RSDK Joint Position Example: Keyboard Control

    Use your dev machine's keyboard to control joint positions.

    Each key corresponds to increasing or decreasing the angle
    of a joint on one of Baxter's arms. Each arm is represented
    by one side of the keyboard and inner/outer key pairings
    on each row for each joint.
    """
    epilog = """
See help inside the example with the '?' key for key bindings.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("rsdk_joint_position_keyboard")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    def clean_shutdown():
        print("\nExiting example...")
        if not init_state:
            print("Disabling robot...")
            rs.disable()
    rospy.on_shutdown(clean_shutdown)

    print("Enabling robot... ")
    rs.enable()

    map_keyboard()
    print("Done.")


if __name__ == '__main__':
    main()
