#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import PoseStamped

def callback(msg):
    try:
        trans = tf2_buffer.lookup_transform(frame_id, child_frame_id, rospy.Time(0))
        pose_transformed = tf2_geometry_msgs.do_transform_pose(msg, trans)
        pose_transformed.header.frame_id = child_frame_id
    except:
        pose_transformed = msg
    pub.publish(pose_transformed)    
    

if __name__ == '__main__':
    rospy.init_node('tf_pose')

    frame_id = rospy.get_param('~frame_id')
    child_frame_id = rospy.get_param('~child_frame_id')
    sub_pose_topic = rospy.get_param('~sub_pose_topic')
    pub_pose_topic = rospy.get_param('~pub_pose_topic')
    
    sub = rospy.Subscriber(sub_pose_topic, PoseStamped, callback)
    pub = rospy.Publisher(pub_pose_topic, PoseStamped, queue_size=10)

    tf2_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf2_buffer)

    rospy.spin()
    