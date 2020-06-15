#!/usr/bin/env python

import rospy
import numpy as np
import tf
from ur5_robot.srv import TfPair


def distance_tf_pair(req):
    listener = tf.TransformListener()
    listener.waitForTransform(req.a, req.b, rospy.Time(), rospy.Duration(4.0))
    while not rospy.is_shutdown():
        try:
            now = rospy.Time.now()
            listener.waitForTransform(req.a, req.b, now, rospy.Duration(4.0))
            (trans, rot) = listener.lookupTransform(req.a, req.b, now)
            #print(rot)
            return trans[0], trans[1], trans[2]
        except rospy.ServiceException:
            print("Service call failed:")

def distance_between_tfs_server():
    rospy.init_node('tf_transformation')
    s = rospy.Service('distance_between_tfs', TfPair, distance_tf_pair)
    print("Ready to compute transformation.")
    rospy.spin()

if __name__ == "__main__":
    distance_between_tfs_server()
