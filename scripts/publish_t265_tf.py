#!/usr/bin/env python 
# to use this python script install
# pip install pyyaml

import yaml
import rospy
import tf
import tf2_ros
import numpy as np
import geometry_msgs.msg
from tf.transformations import quaternion_from_matrix, translation_from_matrix, concatenate_matrices, translation_matrix

# return an homogeneous transformation from the yaml file resulted from calibration
# @param file: path to the yaml file that contains the requested homogeneous transformation
# @param camera_name: name of the camera for the homogeneous transformation (ex. cam0, cam1, cam2, ...)
# @param transform_name: name of the transformation insede the yaml file that need to be retrieved (ex. T_cam_imu, T_cn_cnm1, ...) 
# @return the homogeneous trasnformation requested as a matrix
def getTransformation(file, camera_name, transfrom_name):
    with open(file, 'r') as config_file:
        try:
            # load the yaml file
            file_params = yaml.safe_load_all(config_file)
            for params in file_params:
                # extract the part of the yaml file relatica to 'camera_name'
                cam_params = params[camera_name]
                # extract the part of yaml file, inside the camera specified above, relative to the transformation to obtain
                H_transformation = cam_params[transfrom_name]
                # return the given homogeneous transformation
                return H_transformation
        except yaml.YAMLError as exc:
            print("An error occoured reagarding the YAML file",exc)
        
def buildStaticTransform(transform, base_frame, child_frame):
    # quaternion vector from homogeneous matrix
    quat_cam_imu = quaternion_from_matrix(transform)
    # translation vector
    tran_cam_imu = translation_from_matrix(transform)
    
    # build static stransformation stamped 
    static_transform_cam_imu = geometry_msgs.msg.TransformStamped()
    # filling the header
    static_transform_cam_imu.header.stamp = rospy.Time.now()
    static_transform_cam_imu.header.frame_id = base_frame
    static_transform_cam_imu.child_frame_id = child_frame
    # filling the translation vector
    static_transform_cam_imu.transform.translation.x = tran_cam_imu[0]
    static_transform_cam_imu.transform.translation.y = tran_cam_imu[1]
    static_transform_cam_imu.transform.translation.z = tran_cam_imu[2]
    # filling the quaternion vector
    static_transform_cam_imu.transform.rotation.x = quat_cam_imu[0]
    static_transform_cam_imu.transform.rotation.y = quat_cam_imu[1]
    static_transform_cam_imu.transform.rotation.z = quat_cam_imu[2]
    static_transform_cam_imu.transform.rotation.w = quat_cam_imu[3]
    # return static transformation
    return static_transform_cam_imu
    

if __name__ == '__main__':
    ## read parameter from the launch file
    rospy.init_node("publish_camera_imu_tf_from_yaml")
    imu_file_path = rospy.get_param("~imu_cam_calib_path")
    base_frame = rospy.get_param("~base_frame")
    child_frame = rospy.get_param("~child_frame_prefix")
    num_cameras = rospy.get_param("~num_cameras")
    is_pub_cameras_tf = rospy.get_param("~publish_cameras_tf")
    is_pub_odom_tf = rospy.get_param("~publish_odom_tf")
    I = np.identity(4)

    T_imu_cams = []
    ## load imu_camera yaml file and read transformation cam to imu
    for i in range(num_cameras):
        T_cam_imu = getTransformation(imu_file_path, "cam"+str(i), "T_cam_imu")
        T_imu_cam = np.linalg.inv(T_cam_imu)
        T_imu_cams.append(T_imu_cam)

    if(num_cameras == 2):
        # t265 camera - find the middle point of the two cameras
        # get translation vector from each camera
        t_cam1 = translation_from_matrix(T_imu_cams[0])
        t_cam2 = translation_from_matrix(T_imu_cams[1])
        # get vector between fisheye camera 1 and fisheye camera2
        t_cam1_cam2 = t_cam2 - t_cam1
        # get vector from fisheye camera 1 to the middle point of the previous vector (odometry reference)
        t_cam_odom = t_cam1_cam2 / 2
        # build homogeneous transformation between IMU and the just calculated odometry reference = T_imu_camera1 * T_cam_odom
        T_imu_cam_odom = concatenate_matrices(translation_matrix(t_cam_odom), T_imu_cams[0])

    ## publish T_imu_cams and T_imu_cam_odom transformations as TF static transfroms
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transforms = []
    if(is_pub_cameras_tf):
        for i,transform in enumerate(T_imu_cams):
            T_static_imu_cam = buildStaticTransform(transform,base_frame, child_frame + "_fisheye"+str(i))
            # stack all the rtansforomation in a list because broadcaster is latched to latched to /tf_static so only one stansform can be published at the time
            static_transforms.append(T_static_imu_cam)

    if(is_pub_odom_tf and num_cameras==2):
        T_static_imu_camera_odom = buildStaticTransform(T_imu_cam_odom,base_frame, child_frame + "_odom")
        static_transforms.append(T_static_imu_camera_odom)
    
    # publish all the transformations
    broadcaster.sendTransform(static_transforms)

    ## ROS spin until the end of the program
    rospy.spin()