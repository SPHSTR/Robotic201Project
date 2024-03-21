#!/usr/bin/env python3
# Copyright 2011 Brown University Robotics.
# Copyright 2017 Open Source Robotics Foundation, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import sys

import geometry_msgs.msg
from std_msgs.msg import Float64MultiArray
import rclpy

if sys.platform == "win32":
    import msvcrt
else:
    import termios
    import tty


msg = """
This node takes keypresses from the keyboard and publishes them
as Twist messages. It works best with a US keyboard layout.
---------------------------
Moving around:
        w    
   a    s    d

q/z : increase/decrease max speeds by 10%

CTRL-C to quit
"""

moveBindings = {
    "w": (1, 0, 0, 0),
    "a": (0, 0, 0, 1),
    "d": (0, 0, 0, -1),
    "s": (-1, 0, 0, 0),
}

speedBindings = {
    "q": (1.1, 1.0),
    "e": (0.9, 1.0),
}

    # - LPurpleArm
    # - RPurpleArm

    # - LGreenCube
    # - RGreenCube

    # - LOrangeArm
    # - ROrangeArm

    # - FLPinkMiniArm
    # - RLPinkMiniArm
    # - FRPinkMiniArm
    # - RRPinkMiniArm
jointBindings = {
    
    "y": (0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    "h": (0.0, 0.0, -1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    "u": (0.0, 0.0, 0.0, 0.0, 0.50, 0.50, 0.0, 0.0, 0.0, 0.0),
    "j": (0.0, 0.0, 0.0, 0.0, -0.50, -0.50, 0.0, 0.0, 0.0, 0.0),
    "i": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 1.0, -1.0, 1.0),
    "k": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 1.0, -1.0)
    
}


def getKey(settings):
    if sys.platform == "win32":
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    if sys.platform == "win32":
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == "win32":
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)


def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node("teleop_twist_keyboard")
    pub = node.create_publisher(geometry_msgs.msg.Twist, "cmd_vel", 10)
    pub1 = node.create_publisher(Float64MultiArray, "/gripper_controller/commands", 10)

    speed = 1.5
    turn = 1.5
    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0
    status = 0.0
    j1 = 0.0
    j2 = 0.0
    j3 = 0.0
    j4 = 0.0
    j5 = 0.0
    j6 = 0.0
    j7 = 0.0
    j8 = 0.0
    j9 = 0.0
    j10 = 0.0

    try:
        print(msg)
        print(vels(speed, turn))
        while True:
            key = getKey(settings)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in jointBindings.keys():
                j1 = jointBindings[key][0]
                j2 = jointBindings[key][1]
                j3 = jointBindings[key][2]
                j4 = jointBindings[key][3]
                j5 = jointBindings[key][4]
                j6 = jointBindings[key][5]
                j7 = jointBindings[key][6]
                j8 = jointBindings[key][7]
                j9 = jointBindings[key][8]
                j10 =jointBindings[key][9]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print(vels(speed, turn))
                if status == 14:
                    print(msg)
                status = (status + 1) % 15
            else:
                x = 0.0
                y = 0.0
                z = 0.0
                th = 0.0
                j1 = 0.0
                j2 = 0.0
                j3 = 0.0
                j4 = 0.0
                j5 = 0.0
                j6 = 0.0
                j7 = 0.0
                j8 = 0.0
                j9 = 0.0
                j10 =0.0
                if key == "\x03":
                    break

            twist = geometry_msgs.msg.Twist()
            twist.linear.x = x * speed
            twist.linear.y = y * speed
            twist.linear.z = z * speed
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = th * turn
            pub.publish(twist)

            Float1 = Float64MultiArray()
            Float1.data = [j1,j2,j3,j4,j5,j6,j7,j8,j9,j10]
            pub1.publish(Float1)

    except Exception as e:
        print(e)

    finally:
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

        Float1 = Float64MultiArray()
        Float1.data = [j1,j2,j3,j4,j5,j6,j7,j8,j9,j10]
        pub1.publish(Float1)

        restoreTerminalSettings(settings)


if __name__ == "__main__":
    main()
