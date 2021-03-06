#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy

from geodesy.utm import fromMsg, fromLatLong
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoseStamped

def callback(msg, center):
    gps_pose = PoseStamped()

    gps_position = fromMsg(msg.pose.position)
    gps_pose.pose.position.x = gps_position.easting - center.easting
    gps_pose.pose.position.y = gps_position.northing - center.northing
    gps_pose.pose.position.z = gps_position.altitude - center.altitude

    gps_pose.pose.orientation = msg.pose.orientation
    gps_pose.header = msg.header

    pub.publish(gps_pose)    
    

if __name__ == '__main__':
    rospy.init_node('nav_pose')

    sub_pose_topic   = rospy.get_param('~sub_pose_topic')
    pub_pose_topic   = rospy.get_param('~pub_pose_topic')
    center_latitude  = rospy.get_param('~center_latitude')
    center_longitude = rospy.get_param('~center_longitude')
    center_altitude  = rospy.get_param('~center_altitude')

    center = fromLatLong(center_latitude, center_longitude, center_altitude)
    
    sub = rospy.Subscriber(sub_pose_topic, GeoPoseStamped, lambda msg: callback(msg, center))
    pub = rospy.Publisher(pub_pose_topic, PoseStamped, queue_size=10)

    rospy.spin()
    