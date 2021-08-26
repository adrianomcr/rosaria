#!/usr/bin/env python

"""
8: move forward
2: move backward
4: spin left
6: spin right
7: curve forward and left
9: curve forward and right
1: curve backward and left
3: curve backward and right

q: increase linear and angular (25%)
a: decrease linear and angular (20%)
w: increase linear (25%)
s: decrease linear (20%)
e: increase angular (25%)
d: decrease angular (20%)
"""

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty


moveBindings = {
                '1':(-1, -1),
                '2':(-1, +0),
                '3':(-1, +1),
                '4':(+0, +1),
                '6':(+0, -1),
                '7':(+1, +1),
                '8':(+1, +0),
                '9':(+1, -1),
               }


speedBindings = {
                 'q':(1.25, 1.25),
                 'a':(0.80, 0.80),
                 'w':(1.25, 1.00),
                 's':(0.80, 1.00),
                 'e':(1.00, 1.25),
                 'd':(1.00, 0.80),
                }


def getKey():
    # print "a"
    tty.setraw(sys.stdin.fileno())
    # print "b"
    a, _, _ = select.select([sys.stdin], [], [], 1.0/20.0)
    # print "c"
    if(a):
        key = sys.stdin.read(1)
    else:
        key = '5'
    # print "d"
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    # print "e"
    return key


def vels(speed,turn):
    return "\nlinear velocity: %s\nangular velocity: %s" % (speed,turn)


if __name__=="__main__":
    
    settings = termios.tcgetattr(sys.stdin)
    
    pub = rospy.Publisher('/RosAria/cmd_vel_raw', Twist, queue_size=1)
    rospy.init_node('teleop_numeric_node')
    
    init_vel = input("\nType the value of initial velocity (1.0): ")
    
    speed = rospy.get_param("~speed", float(init_vel))
    turn = rospy.get_param("~turn", float(init_vel))
    lin = 0
    ang = 0
    
    try:
        
        print(vels(speed,turn))
        
        while(True):
            
            key = getKey()
            
            if key in moveBindings.keys():
                lin = moveBindings[key][0]
                ang = moveBindings[key][1]
            
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                print(vels(speed,turn))
            
            else:
                lin = 0
                ang = 0
                if (key == '\x03'):
                    break
            
            twist = Twist()
            twist.linear.x = lin*speed; twist.linear.y = 0 ; twist.linear.z = 0;
            twist.angular.x = 0;        twist.angular.y = 0; twist.angular.z = ang*turn
            pub.publish(twist)
    
    except Exception as e:
        print(e)
    
    finally:
        
        twist = Twist()
        twist.linear.x = 0;  twist.linear.y = 0;  twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)
        
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
