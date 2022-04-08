#!/usr/bin/env python 
# to use this python script install
# pip install pyyaml

import yaml
import rospy
import tf
import tf2_ros
import numpy as np
import geometry_msgs.msg
from tf.transformations import quaternion_from_matrix, translation_from_matrix, concatenate_matrices

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
    rospy.init_node("publish_cameras_tf_from_yaml")
    imu_file_path = rospy.get_param("~imu_cam_calib_path")
    cam_file_path = rospy.get_param("~cam_cam_calib_path")
    base_frame = rospy.get_param("~base_frame")
    child_frame = rospy.get_param("~child_frame_prefix")
    num_cameras = rospy.get_param("~num_cameras")
    I = [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0],[0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]
    ## load imu_camera yaml file and read transformation imu to cam0 
    T_cam_imu = getTransformation(imu_file_path, "cam0", "T_cam_imu")

    ## load camera_Camera yaml file and read all the transformation of the camera chain
    T_cameras = []
    for i in range(1,num_cameras): # number between 1 and num_cameras -1
        T_cam_cam = getTransformation(cam_file_path, "cam"+str(i), "T_cn_cnm1")
        T_cam_cam = concatenate_matrices(I,T_cam_cam)
        T_cameras.append(T_cam_cam)
        print("T_cam"+str(i)+"_cam"+str(i-1))
        print(T_cam_cam)
    # at the end: T_cameras = [T_1_0, T_2_1, T_3_2, T_4_3, T_5_4]

    ## build T_cam*_imu transformations
    T_cam_imu = concatenate_matrices(I,T_cam_imu)
    T_cameras_imu = [T_cam_imu]
    print("T_cam0_imu")
    print(T_cam_imu)
    for i,transform in enumerate(T_cameras):
        # new T_cam(i)_imu = T_cam(i)_cam(i-1) * T_cam(i-1)_imu 
        T_cam_imu_temp = concatenate_matrices(transform, T_cameras_imu[-1])
        T_cameras_imu.append(T_cam_imu_temp)
        print("T_cam"+str(i+1)+"_imu")
        print(T_cam_imu_temp)
    # at the end T_cameras_imu = [T_cam0_imu, T_cam1_imu, T_cam2_imu, T_cam3_imu, T_cam4_imu, T_cam5_imu]

    ## publish T_cameras_imu transformations as TF static transfroms
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transforms = []
    for i,transform in enumerate(T_cameras_imu):
        T_static_imu_cam = buildStaticTransform(transform,base_frame, child_frame + "_"+str(i))
        # stack all the rtansforomation in a list because broadcaster is latched to latched to /tf_static so only one stansform can be published at the time
        static_transforms.append(T_static_imu_cam)

    # publish all the transformations
    broadcaster.sendTransform(static_transforms)

    # ROS spin until the end of the program
    rospy.spin()