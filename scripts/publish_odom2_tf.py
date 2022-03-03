#!/usr/bin/env python 
import rospy
import tf
import math
from tf.transformations import concatenate_matrices, quaternion_matrix, quaternion_from_matrix, translation_matrix, translation_from_matrix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np

#camera_name = ""
#prefix_odom = ""
 
def handle_camera_odom(msg_odom):
    global camera_name, prefix_odom, cam_odom_pub, listener
    try:
        if(listener.frameExists('/'+prefix_odom+'_odom_frame')):
            #listener.waitForTransform('/odom','/'+ 'base_link', rospy.Time(0), rospy.Duration(30.0))
            #listener.waitForTransform('/odom','/'+ prefix_odom +'_odom_frame', rospy.Time(0), rospy.Duration(4.0))
            (trans,rot) = listener.lookupTransform('/odom','/'+ prefix_odom +'_odom_frame', rospy.Time(0))

            # perform transformation
            old_pose = PoseStamped()
            old_pose.header.frame_id = '/'+prefix_odom+'_odom_frame'
            old_pose.pose = msg_odom.pose.pose
            
            transformed_pose = listener.transformPose('/odom',old_pose)
            
            print("-"*5)
            print("old_pose 2",old_pose)
            print("transformed_pose 2", transformed_pose)
            print("-"*5)

            # publish new message
            odo = msg_odom
            odo.pose.pose = transformed_pose.pose
            #odo.pose.pose.position.x = pose2odom[0]
            #odo.pose.pose.position.y = pose2odom[1]
            #odo.pose.pose.position.z = pose2odom[2]
            odo.header.frame_id = "odom"
            #odo.child_frame_id = "base_link"
            cam_odom_pub.publish(odo)
        else:
            msg_odom.header.frame_id = "odom"
            cam_odom_pub.publish(msg_odom)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass
        #print("Error in getting the transformation")
    



if __name__ == '__main__':
    #camera_name = rospy.get_param("~camera_name") # default camera1
    #prefix_odom = rospy.get_param("~prefix_odom") # default cam1
    #node_name = rospy.get_param("~node_name") # default odom_tf_listener

    camera_name = "camera2"
    prefix_odom = "cam2"
    node_name = "odom2_tf_listener"

    rospy.init_node(node_name) 
    listener = tf.TransformListener()

    cam_odom_pub = rospy.Publisher('/'+prefix_odom+'/odom/sample_throttled', Odometry, queue_size=100)
    cam_odom_sub = rospy.Subscriber('/'+camera_name+'/odom/sample_throttled', Odometry, handle_camera_odom, queue_size=100)
 
    rate = rospy.Rate(200.0)
    while not rospy.is_shutdown():
    #    pass
    #    #try:
    #    #    (trans,rot) = listener.lookupTransform('/'+camera_name+'_odom_frame', '/'+camera_name+'_pose_frame', rospy.Time(0))
    #    #except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #    #    continue
    #    
        br = tf.TransformBroadcaster()
    #    # send out the pose of the turtle in the form of a transform
    #    #br.sendTransform(trans,
    #    #                 rot,
    #    #                 rospy.Time.now(),
    #    #                 prefix_odom+"_pose_frame",
    #    #                 prefix_odom+"_odom_frame")
    #    br = tf.TransformBroadcaster()
    #    # send out the pose of the turtle in the form of a transform
        #br.sendTransform((0.0935, 0, 0),
        #                 tf.transformations.quaternion_from_euler(0 , 0, 0.0),
        #                 rospy.Time.now(),
        #                 "/"+camera_name+"_odom_frame",
        #                 "/odom")
        # # implement solution of ROS answer
        try:
            #if(listener.frameExists("/base_link") and listener.frameExists('/'+camera_name+'_odom_frame')):
            #listener.waitForTransform('/odom','/'+camera_name+'_odom_frame', rospy.Time(0), rospy.Duration(4.0))
            (trans_cam_odom,rot_cam_odom) = listener.lookupTransform('/odom','/'+camera_name+'_odom_frame', rospy.Time(0))

            #listener.waitForTransform('/odom','/base_link', rospy.Time(0), rospy.Duration(30.0))
            #(trans_odom_bl,rot_odom_bl) = listener.lookupTransform('/odom','/base_link', rospy.Time(0))

            T_cam_odom = translation_matrix(trans_cam_odom)
            R_cam_odom = quaternion_matrix(rot_cam_odom)
            #R_odom_bl = quaternion_matrix(rot_odom_bl)

            #H_odom_pose = concatenate_matrices(T_cam_odom, R_odom_bl)
            H_odom_pose = concatenate_matrices(T_cam_odom, R_cam_odom)

            br.sendTransform(translation_from_matrix(H_odom_pose),
                             quaternion_from_matrix(H_odom_pose),
                             rospy.Time.now(),
                             '/'+ prefix_odom +'_odom_frame',
                             "odom")
            #else:
            #    print("Some frames do not exist")

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("Error tf")
            continue
 

    #    br.sendTransform((0.0935, 0, 0),
    #                      tf.transformations.quaternion_from_euler(0 , 0, 0.0),
    #                      rospy.Time.now(),
    #                      "cam2_pose_frame",
    #                      "base_link")
        rate.sleep()