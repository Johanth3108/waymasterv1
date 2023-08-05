#include <ros.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>

ros::NodeHandle nh;
nav_msgs::Odometry odom;
ros::Publisher odom_pub("rodom", &odom);

void setup() {
  nh.initNode();
  nh.advertise(odom_pub);
}

void loop() {
  // put your main code here, to run repeatedly:
  odom.header.stamp = nh.now();
  
  odom_pub.publish(&odom);
  nh.spinOnce();
  delay(1000);
}
