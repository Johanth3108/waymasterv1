#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16
from math import pi

def callback(data):
    # rospy.loginfo(data.data)
    encoder_ticks = 206.25
    wheel_dia = 0.067
    circum = wheel_dia*pi
    dist_per_rev = circum

    distance = (data.data/encoder_ticks)*dist_per_rev
    print(distance)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('values_printer', anonymous=True)

    rospy.Subscriber("left_wheel", Int16, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()