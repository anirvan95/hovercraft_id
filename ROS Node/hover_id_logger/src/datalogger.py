#!/usr/bin/env python

from __future__ import print_function

import roslib
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
import sys, select, termios, tty
import xlsxwriter
import scipy.io


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

optiPose = []
hovePoseStamped = []

def getOptiPose(data):
    global optiPose
    optiPose = [data.pose.x,data.pose.y,data.pose.y]

def getHoverFeeback(data):
    global hovPoseStamped
    hovPoseStamped = [data.header.stamp, data.pose.position.x,data.pose.position.y,data.pose.orientation.x,data.pose.orientation.y]

def stopNode():
  print("Emergency Stop")

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    pub = rospy.Publisher('cmd_thu', Pose, queue_size = 1)
    rospy.Subscriber("\Robot\pose", String, getOptiPose)
    rospy.Subscriber("hoverstats", PoseStamped, getHoverFeeback)
    rospy.init_node('ident_logger')
    r = rospy.Rate(20)
    data = scipy.io.loadmat('pwmdataId_spec_2.mat')
    timeData = data['pwmdataId']['t']
    pwmData = data['pwmdataId']['PWM']
    t = timeData[0][0]
    pwm = pwmData[0][0]

    t0 = rospy.get_rostime()
    rospy.on_shutdown(stopNode)
    rospy.sleep(10)
    print("Sending motor commands")
    for i in range(0,t.size):
        tr = rospy.get_rostime()
        data = []
        while (tr-t0<rospy.Duration(t[0][i])):

            motorCommand = Pose()
            motorCommand.position.z = 1375
            motorCommand.position.x = pwm[0, i]
            motorCommand.position.y = pwm[1,i]
            motorCommand.orientation.x = pwm[2,i]
            motorCommand.orientation.y = pwm[3,i]
            pub.publish(motorCommand)
            data.append(i,[double(tr-t0),pwm[0,i],pwm[1,i],pwm[2,i],pwm[3,i],optiPose,hovePoseStamped])
            r.sleep()
            tr = rospy.get_rostime()

    print("Stopped")
    np.savetxt("DatapwmdataId_spec_2.csv", np.array(data), delimiter=",")
        