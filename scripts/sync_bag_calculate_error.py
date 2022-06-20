#!/usr/bin/env python 
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import message_filters
from std_msgs.msg import Int32, Float32, Float32MultiArray
import math
import numpy as np

# Calculate the MSE of X, Y and Z coordinate between two numpy vector
def mse(pose1, pose2):
    mse = np.square(np.subtract(pose1,pose2)).mean() 
    return mse

def dist(pose1, pose2):
    dist = np.square(np.subtract(pose1,pose2).sum())
    return float(dist)

# calculate absolute distance between the two numpy vector
def abs_dist_vect(pose1,pose2):
    vect_dist = np.abs(np.subtract(pose1,pose2))
    return vect_dist

def callback(cam1, cam2, cam3, ekf, optitrack):
    # The callback processing the odometry messages that arrived at approximately the same time
    time_now = rospy.Time(0)
    cam1.header.stamp = time_now
    cam2.header.stamp = time_now
    cam3.header.stamp = time_now
    ekf.header.stamp = time_now
    optitrack.header.stamp = time_now

    # publish the rostopic at the same time
    cam1_pub.publish(cam1)
    cam2_pub.publish(cam2)
    cam3_pub.publish(cam3)
    ekf_pub.publish(ekf)
    optitrack_pub.publish(optitrack)

    ## CAM 1
    # calculate MSE of the position wrt the ground truth of the Optitrack
    reference_pose = np.array([optitrack.pose.position.x, optitrack.pose.position.y, optitrack.pose.position.z])
    curr_pose = np.array([cam1.pose.pose.position.x, cam1.pose.pose.position.y, cam1.pose.pose.position.z])
    mse_cam1 = mse(curr_pose, reference_pose)
    # calculate distance for each dimentions
    cam1_abs_dist_vector = abs_dist_vect(curr_pose, reference_pose)
    min = np.min(cam1_abs_dist_vector)
    max = np.max(cam1_abs_dist_vector)
    avg = np.mean(cam1_abs_dist_vector)
    cam1_abs_dist_vector = np.append(cam1_abs_dist_vector, [min, max, avg])
    # calculate eucledian distance
    cam1_ecl_dist = dist(curr_pose, reference_pose)

    ## CAM 2
    # calculate MSE of the position wrt the ground truth of the Optitrack
    curr_pose = np.array([cam2.pose.pose.position.x, cam2.pose.pose.position.y, cam2.pose.pose.position.z])
    mse_cam2 = mse(curr_pose, reference_pose)
    # calculate distance for each dimentions
    cam2_abs_dist_vector = abs_dist_vect(curr_pose, reference_pose)
    min = np.min(cam2_abs_dist_vector)
    max = np.max(cam2_abs_dist_vector)
    avg = np.mean(cam2_abs_dist_vector)
    cam2_abs_dist_vector=np.append(cam2_abs_dist_vector, [min, max, avg])
    # calculate eucledian distance
    cam2_ecl_dist = dist(curr_pose, reference_pose)

    ## CAM 3
    # calculate MSE of the position wrt the ground truth of the Optitrack
    curr_pose = np.array([cam3.pose.pose.position.x, cam3.pose.pose.position.y, cam3.pose.pose.position.z])
    mse_cam3 = mse(curr_pose, reference_pose)
    # calculate distance for each dimentions
    cam3_abs_dist_vector = abs_dist_vect(curr_pose, reference_pose)
    min = np.min(cam3_abs_dist_vector)
    max = np.max(cam3_abs_dist_vector)
    avg = np.mean(cam3_abs_dist_vector)
    cam3_abs_dist_vector = np.append(cam3_abs_dist_vector, [min, max, avg])
    # calculate eucledian distance
    cam3_ecl_dist = dist(curr_pose, reference_pose)

    ## EKF
    # calculate MSE of the position wrt the ground truth of the Optitrack
    curr_pose = np.array([ekf.pose.pose.position.x, ekf.pose.pose.position.y, ekf.pose.pose.position.z])
    mse_ekf = mse(curr_pose, reference_pose)
    # calculate distance for each dimentions
    ekf_abs_dist_vector = abs_dist_vect(curr_pose, reference_pose)
    min = np.min(ekf_abs_dist_vector)
    max = np.max(ekf_abs_dist_vector)
    avg = np.mean(ekf_abs_dist_vector)
    ekf_abs_dist_vector = np.append(ekf_abs_dist_vector, [min, max, avg])
    # calculate eucledian distance
    ekf_ecl_dist = dist(curr_pose, reference_pose)


    ## Publish absolute distance for each dimension - for each camera and EKF
    dist_vect = Float32MultiArray()
    dist_vect.data = cam1_abs_dist_vector
    cam1_abs_dist_pub.publish(dist_vect)
    dist_vect.data = cam2_abs_dist_vector
    cam2_abs_dist_pub.publish(dist_vect)
    dist_vect.data = cam3_abs_dist_vector
    cam3_abs_dist_pub.publish(dist_vect)
    dist_vect.data = ekf_abs_dist_vector
    ekf_abs_dist_pub.publish(dist_vect)

    ## Publish MSE for each camera and EKF
    cam1_error_pub.publish(mse_cam1)
    cam2_error_pub.publish(mse_cam2)
    cam3_error_pub.publish(mse_cam3)
    ekf_error_pub.publish(mse_ekf)

    ## Publish eucledian distance for each camera and EKF
    cam1_dist_error_pub.publish(cam1_ecl_dist)
    cam2_dist_error_pub.publish(cam2_ecl_dist)
    cam3_dist_error_pub.publish(cam3_ecl_dist)
    ekf_dist_error_pub.publish(ekf_ecl_dist)


if __name__ == '__main__':

    rospy.init_node("Calculate error") 
    
    cam1_pub = rospy.Publisher("/sync/cam1/odom/sample_throttled", Odometry, queue_size=10)
    cam2_pub = rospy.Publisher("/sync/cam2/odom/sample_throttled", Odometry, queue_size=10)
    cam3_pub = rospy.Publisher("/sync/cam3/odom/sample_throttled", Odometry, queue_size=10)
    ekf_pub = rospy.Publisher("/sync/odometry/filtered", Odometry, queue_size=10)
    optitrack_pub = rospy.Publisher("/sync/drone_2/pose", PoseStamped, queue_size=10)

    cam1_error_pub = rospy.Publisher("/mse/cam1/odom/sample_throttled", Float32, queue_size=10)
    cam2_error_pub = rospy.Publisher("/mse/cam2/odom/sample_throttled", Float32, queue_size=10)
    cam3_error_pub = rospy.Publisher("/mse/cam3/odom/sample_throttled", Float32, queue_size=10)
    ekf_error_pub = rospy.Publisher("/mse/odometry/filtered", Float32, queue_size=10)

    cam1_dist_error_pub = rospy.Publisher("/dist/cam1/odom/sample_throttled", Float32, queue_size=10)
    cam2_dist_error_pub = rospy.Publisher("/dist/cam2/odom/sample_throttled", Float32, queue_size=10)
    cam3_dist_error_pub = rospy.Publisher("/dist/cam3/odom/sample_throttled", Float32, queue_size=10)
    ekf_dist_error_pub = rospy.Publisher("/dist/odometry/filtered", Float32, queue_size=10)

    cam1_abs_dist_pub = rospy.Publisher("/abs_dist/cam1/odom/sample_throttled", Float32MultiArray, queue_size=10)
    cam2_abs_dist_pub = rospy.Publisher("/abs_dist/cam2/odom/sample_throttled", Float32MultiArray, queue_size=10)
    cam3_abs_dist_pub = rospy.Publisher("/abs_dist/cam3/odom/sample_throttled", Float32MultiArray, queue_size=10)
    ekf_abs_dist_pub = rospy.Publisher("/abs_dist/odometry/filtered", Float32MultiArray, queue_size=10)


    cam1_sub = message_filters.Subscriber("/cam1/odom/sample_throttled", Odometry)
    cam2_sub = message_filters.Subscriber("/cam2/odom/sample_throttled", Odometry)
    cam3_sub = message_filters.Subscriber("/cam3/odom/sample_throttled", Odometry)
    ekf_sub = message_filters.Subscriber("/odometry/filtered", Odometry)
    optitrack_sub = message_filters.Subscriber("/drone_2/pose", PoseStamped)

    ts = message_filters.ApproximateTimeSynchronizer([cam1_sub, cam2_sub, cam3_sub, ekf_sub, optitrack_sub], 10, 0.1)
    ts.registerCallback(callback)
    
    while not rospy.is_shutdown():
        rospy.spin()