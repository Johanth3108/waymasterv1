#!/usr/bin/env python  
import rospy
from std_msgs.msg import Int16
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

def handle_turtle_pose(msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((0.5, 0, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "base_link",
                     "odom")
    print("tf sending")

    odom_quat = tf.transformations.quaternion_from_euler(0, 0, 0)
    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    # set the position
    odom.pose.pose = Pose(Point(0.5, 0, 0.), Quaternion(*odom_quat))

    # set the velocity
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(0.5, 0, 0), Vector3(0, 0, 0))

    # publish the message
    odom_pub.publish(odom)

    last_time = current_time
    # br.sendTransform(child=,parent=)

if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')
    # turtlename = rospy.get_param('~turtle')
    current_time = rospy.Time.now()
    last_time = rospy.Time.now()
    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
    rospy.Subscriber('left_wheel',
                     Int16,
                     handle_turtle_pose)
    rospy.spin()