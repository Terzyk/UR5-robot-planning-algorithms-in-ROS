#!/usr/bin/env python

# !/usr/bin/env python

import rospy
import tf
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
#from ros_object_detection.msg import BoundingBoxes
import sensor_msgs.msg
from cv_bridge import CvBridge
import cv2
import pickle


class Callbacks:
    def __init__(self):
        pass


    def tfs(self):

        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=5)
        rospy.init_node('tf_targets')
        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            try:
                with open("data.pkl", "rb") as a_file:
                    path = pickle.load(a_file)
            except:
                path=[]

            for i in range(0, len(path)):

                t = geometry_msgs.msg.TransformStamped()
                t.header.frame_id = "world"
                t.header.stamp = rospy.Time.now()
                t.child_frame_id = "target"+str(i)
                t.transform.rotation.x = 0
                t.transform.rotation.y = 0
                t.transform.rotation.z = 0
                t.transform.rotation.w = 1.0
                t.transform.translation.x = path[i][0]
                t.transform.translation.y = path[i][1]
                t.transform.translation.z = path[i][2]
                tfm = tf2_msgs.msg.TFMessage([t])
                self.pub_tf.publish(tfm)
                rate.sleep()


if __name__ == '__main__':
    try:
        callbacks = Callbacks()
        callbacks.tfs()
    except rospy.ROSInterruptException:
        pass

