#!/usr/bin/env python 
import rospy
import tf
import math
from nav_msgs.msg import Odometry



#def handle_rovio_odom(msg_odom):
#    global br
#    position = msg_odom.pose.pose.position
#    br.sendTransform((0.0, 0.0 , 0.0),
#                         tf.transformations.quaternion_from_euler( math.radians(0.0), math.radians(90), math.radians(0.0)),
#                         rospy.Time.now(),
#                         "abs_correction", "odom")
    #br.sendTransform((0.0, 0.0 , 0.0),
    #                     tf.transformations.quaternion_from_euler( math.radians(-90.0), math.radians(0), math.radians(-90.0)),
    #                     rospy.Time.now(),
    #                     "abs_correction", "odom")

if __name__ == '__main__':
    rospy.init_node('fixed_mynteye_realsense_tf_broadcaster')
    #rospy.Subscriber( '/rovio/odometry', Odometry, handle_rovio_odom, queue_size=100)
    #rospy.Subscriber( '/camera/odom/sample', Odometry, handle_rovio_odom, queue_size=100)
    
    br = tf.TransformBroadcaster()

    rate = rospy.Rate(200.0)
    #rospy.spin()
    while not rospy.is_shutdown():
        br.sendTransform((0.0, 0.0 , 0.0),
                         tf.transformations.quaternion_from_euler( math.radians(0.0), math.radians(0), math.radians(0.0)),
                         rospy.Time.now(),
                         "abs_correction", "odom")
        rate.sleep()