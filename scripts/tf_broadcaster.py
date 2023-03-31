#!/usr/bin/env python

import rospy
import time
import tf
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('tf_end_effector')
    listener = tf.TransformListener()
    broadcaster = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)

while not rospy.is_shutdown():    
    try:
        now = rospy.Time(0)
        (trans, rot) =  listener.lookupTransform('/world', '/end_effector', now)
        broadcaster.sendTransform(trans, rot, rospy.Time.now(), '/end_effector', '/world')
    except (tf.LookupException, tf.ConnectivityException):
        continue

    rate.sleep()
