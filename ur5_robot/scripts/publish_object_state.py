#!/usr/bin/env python

import roslib
roslib.load_manifest('ur5_robot')
import rospy
import tf
import tf.msg
import geometry_msgs.msg
import gazebo_msgs.srv

class FixedTFBroadcaster:

  def __init__(self):
    self.pub_tf = rospy.Publisher("/tf", tf.msg.tfMessage, queue_size=1)
    
    # publish transform from world to object
   
    t2 = geometry_msgs.msg.TransformStamped()
    t2.header.frame_id = "world"
    t2.child_frame_id = "/object_link"
    
    r = rospy.Rate(10)
    
    while not rospy.is_shutdown():

      
      try:      
        response=model_state('object','world')
      except rospy.ServiceException, e:
        rospy.loginfo("Service call failed: %s", e)
            
      if response.success:
            
        # transform from world to cabinet

        t2.header.stamp = rospy.Time.now()
        t2.transform.translation = response.pose.position
        t2.transform.rotation = response.pose.orientation
        
        tfm2 = tf.msg.tfMessage([t2])
        self.pub_tf.publish(tfm2)
      
      # sleep for 0.1s
      r.sleep()


if __name__ == '__main__':

    rospy.loginfo("Starting my_tf_broadcaster...")
    rospy.init_node('my_tf_broadcaster')
    
    rospy.sleep(5)
    
    rospy.loginfo("Waiting for service gazebo/get_model_state...")
    rospy.wait_for_service('gazebo/get_model_state')
    
    # maybe this should be persistent connection....
    model_state = rospy.ServiceProxy('gazebo/get_model_state', gazebo_msgs.srv.GetModelState)
    
    tfb = FixedTFBroadcaster()
    rospy.spin()
    rospy.loginfo("my_tf_broadcaster died...")
