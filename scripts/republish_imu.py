#!/usr/bin/env python 
# This node is just in charge of inflating the covariance of the IMU by a certain scale factor
# to make it more stable when the drone flys inside the tank
import rospy
import numpy as np
from sensor_msgs.msg import Imu

 
def handle_imu(msg_imu):
    global imu_pub
    kInflateAngVel = 1e6
    kInflateLinAcc = 1e6
    
    # increase angola velocity covariance
    cov_ang_vel = np.array(msg_imu.angular_velocity_covariance)
    new_cov_ang_vel = cov_ang_vel * kInflateAngVel
    msg_imu.angular_velocity_covariance = tuple(new_cov_ang_vel)
    
    # increase linear acceleration covariance
    cov_lin_acc = np.array(msg_imu.linear_acceleration_covariance)
    new_cov_lin_acc = cov_lin_acc * kInflateLinAcc
    msg_imu.linear_acceleration_covariance = tuple(new_cov_lin_acc)
    
    imu_pub.publish(msg_imu)


if __name__ == '__main__':

    rospy.init_node("republish imu") 
    

    imu_pub = rospy.Publisher("/imu/data_inflated", Imu, queue_size=100)
    imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, handle_imu, queue_size=100)
 
    rospy.spin()