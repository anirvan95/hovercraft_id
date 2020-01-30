#!/usr/bin/env python

from __future__ import print_function

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
CTRL-C to quit
"""

moveBindings = {
        'i':(1,0),
        'j':(0,1),
        'l':(0,-1),
        ',':(-1,0),
    }

speedBindings={
        'q':(0.25,0.1),
        'z':(-0.25,-0.1),
    }

motorBindings={
        's':(1,0,0,0,0),#LT
        'x':(-1,0,0,0,0),
        'd':(0,1,0,0,0),#BL
        'c':(0,-1,0,0,0),
        'f':(0,0,1,0,0),#BR
        'v':(0,0,-1,0,0),
        'g':(0,0,0,1,0),#FL
        'b':(0,0,0,-1,0),
        'h':(0,0,0,0,1),#FR
        'n':(0,0,0,0,-1),
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed):
    return "currently:\tspeed %s\t " % (speed)

def arduino_map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    pub = rospy.Publisher('cmd_thu', PoseArray, queue_size = 1)
    rospy.init_node('teleop_twist_keyboard')

    speed = rospy.get_param("~speed", 1.25)
    turn = rospy.get_param("~turn", 0.5)
    vx = 0.0
    th = 0
    LT = 1220
    BL = 0
    FL = 0
    FR = 0
    BR = 0

    status = 0

    try:
        print(msg)
        print(vels(speed))
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                vx += speed*moveBindings[key][0]
                th += turn*moveBindings[key][1]
                map2 = 3.0
                if(th>=0 and vx >=0):
                    BR = arduino_map(vx, 0.0,map2,1000,1300)+arduino_map(th, 0, map2, 0, 400)
                    BL = arduino_map(vx, 0.0,map2,1000,1300)
                    FR = 1000
                    FL = 1000+arduino_map(th, 0, map2, 0, 400)
                elif(th>=0 and vx <0):
                    BR = 1000+arduino_map(th, 0, map2, 0, 400)
                    BL = 1000
                    FR = arduino_map(-vx, 0.0,map2,1000,1300)
                    FL = arduino_map(-vx, 0.0,map2,1000,1300)+arduino_map(th, 0, map2, 0, 400)
                elif(th<0 and vx >=0):
                    BR = arduino_map(vx, 0.0,map2,1000,1300)
                    BL = arduino_map(vx, 0.0,map2,1000,1300)+arduino_map(-th, 0, map2, 0, 400)
                    FR = 1000+arduino_map(-th, 0, map2, 0, 400)
                    FL = 1000
                elif(th<0 and vx <0):
                    BR = 1000
                    BL = 1000+arduino_map(-th, 0, map2, 0, 400)
                    FR = arduino_map(-vx, 0.0,map2,1000,1300)+arduino_map(-th, 0, map2, 0, 400)
                    FL = arduino_map(-vx, 0.0,map2,1000,1300)

            elif key in speedBindings.keys():
                speed += speedBindings[key][0]
                turn += speedBindings[key][1]
            elif key in motorBindings.keys():
                LT += motorBindings[key][0]
                BL += motorBindings[key][1]
                BR += motorBindings[key][2]
                FL += motorBindings[key][3]
                FR += motorBindings[key][4]

                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                vx = 0
                th = 0
                BL = 1000
                BR = 1000
                FL = 1000
                FR = 1000
                LT = 1000

            if (key == '\x03'):
                break


            motorCommand = Pose()
            motorCommand.position.x = BR
            motorCommand.position.y = BL
            motorCommand.position.z = LT
            motorCommand.orientation.x = FR
            motorCommand.orientation.y = FL
            pub.publish(motorCommand)
            print(vels(speed))
            print("Current commands \tvx: %s\t th: %s" %(vx,th))
            print("Current Motor States: \tTL: %s\t BL %s\t BR %s\t FL %s\t FR %s \n" % (LT,BL,BR,FL,FR))

    except Exception as e:
        print(e)
