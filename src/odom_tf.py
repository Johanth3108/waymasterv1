#! /usr/bin/env python
import rospy
import roslib
from math import sin, cos, pi
from rospy import Time
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose, Point, Vector3
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from std_msgs.msg import Int16
import tf.transformations
import tf2_ros
from geometry_msgs.msg import TransformStamped


class Tf:
    def __init__(self):

        rospy.init_node("diff_tf")
        self.nodename = rospy.get_name()
        rospy.loginfo("%s started" % self.nodename)
        
        #### parameters #######
        self.rate = rospy.get_param('~rate',10.0)  # the rate at which to publish the transform
        self.ticks_meter = float(rospy.get_param('ticks_meter', 980))  # The number of wheel encoder ticks per meter of travel
        self.base_width = float(rospy.get_param('~base_width', 0.21)) # The wheel base width in meters
        
        self.base_frame_id = rospy.get_param('~base_frame_id','base_link') # the name of the base frame of the robot
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom') # the name of the odometry reference frame
        
        self.encoder_min = rospy.get_param('encoder_min', -32768)
        self.encoder_max = rospy.get_param('encoder_max', 32768)
        self.encoder_low_wrap = rospy.get_param('wheel_low_wrap', (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min )
        self.encoder_high_wrap = rospy.get_param('wheel_high_wrap', (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min )
 
        self.t_delta = rospy.Duration(1.0/self.rate)
        self.t_next = rospy.Time.now() + self.t_delta


        # internal data
        self.enc_left = None        # wheel encoder readings
        self.enc_right = None
        self.robot_width = 0.21
        self.encoder_ticks = 206.25
        self.wheel_dia = 0.067
        self.dist_per_rev = self.wheel_dia*pi

        self.left = 0               # actual values coming back from robot
        self.right = 0
        self.lmult = 0
        self.rmult = 0
        self.prev_lencoder = 0
        self.prev_rencoder = 0
        self.x = 0                  # position in xy plane 
        self.y = 0
        self.th = 0
        self.dx = 0                 # speeds in x/rotation
        self.dr = 0
        self.then = rospy.Time.now()


        self.ard_sub = rospy.Subscriber("/right_wheel", Int16, self.rcallback)
        self.ard_sub = rospy.Subscriber("/left_wheel", Int16, self.lcallback)
        self.odomPub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.odomBroadcaster = tf.TransformBroadcaster()

    def rcallback(self, msg):
        self.enc_right = msg.data

    def lcallback(self, msg):
        self.enc_left = msg.data

    def update(self):
        now = rospy.Time.now()
        if now > self.t_next:
            elapsed = now - self.then
            self.then = now
            elapsed = elapsed.to_sec()

            # calculate odometry
            if self.enc_left == None:
                d_left = 0
                d_right = 0
            else:
                d_left = (self.left - self.enc_left) / self.ticks_meter
                d_right = (self.right - self.enc_right) / self.ticks_meter
                # d_left = (self.enc_left/self.encoder_ticks)*self.dist_per_rev
                # d_right = (self.enc_right/self.encoder_ticks)*self.dist_per_rev
            
            distance_per_wheel = (d_left+d_right)/2
            theta = (d_right-d_left)/self.robot_width

            x = distance_per_wheel*cos(theta)
            y = distance_per_wheel*sin(theta)
            z = 0.0

            roll = 0.0
            pitch = 0.0
            yaw = theta
            self.dx = distance_per_wheel / elapsed
            self.dr = theta / elapsed
            # print(x,"--::--",y,"--::--",theta)

            ########################################################################
            # broadcaster testing
            ########################################################################

            # b = TransformBroadcaster()

            # translation = (0.0, 0.0, 0.0)
            # rotation = (0.0, 0.0, 0.0, 1.0)
            # rate = rospy.Rate(5)
            # x, y = 0.0, 0.0

            # while not rospy.is_shutdown():
            #     if x>=2:
            #         x, y, z = 0.0, 0.0, 0.0
            
            #     x=1
            #     y=1
            #     z+=0.1
            #     translation = (x, y, 0.0)
            #     rotation = tf.transformations.quaternion_from_euler(0.0, 0.0, z)
            #     b.sendTransform(translation, rotation, Time.now(), 'waymaster', '/world')
            #     rate.sleep()

            ########################################################################
            ########################################################################


            transform_msg = TransformStamped()
            transform_msg.header.stamp = now
            transform_msg.header.frame_id = 'world'
            transform_msg.child_frame_id = 'base_link'

            transform_msg.transform.translation.x = x
            transform_msg.transform.translation.y = 1.0
            transform_msg.transform.translation.z = 0.0

            quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
            transform_msg.transform.rotation.x = quat[0]
            transform_msg.transform.rotation.y = quat[1]
            transform_msg.transform.rotation.z = quat[2]
            transform_msg.transform.rotation.w = quat[3]
            
            self.odomBroadcaster.sendTransformMessage(transform_msg)
            print(yaw)

            # odom = Odometry()
            # odom.header.stamp = now
            # odom.header.frame_id = 'odom'
            # odom.pose.pose.position.x = self.x
            # odom.pose.pose.position.y = self.y
            # odom.pose.pose.position.z = 0
            # odom.pose.pose.orientation = quat
            # odom.child_frame_id = 'base_link'
            # odom.twist.twist.linear.x = self.dx
            # odom.twist.twist.linear.y = 0
            # odom.twist.twist.angular.z = self.dr
            # self.odomPub.publish(odom)
            # print(odom)

            # next, we'll publish the odometry message over ROS
            # odom = Odometry()
            # odom.header.stamp = now
            # odom.header.frame_id = "odom"

            # # set the position
            # odom.pose.pose = Pose(Point(x, y, 0.0), Quaternion(*quat))

            # # set the velocity
            # odom.child_frame_id = "base_link"
            # odom.twist.twist = Twist(Vector3(x, y, 0), Vector3(0, 0, theta))

            # # publish the message
            # self.odomPub.publish(odom)
            # print(odom)



    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()


if __name__ == '__main__':

    try:
        diffTf = Tf()
        diffTf.spin()
    except rospy.ROSInterruptException:
        pass
