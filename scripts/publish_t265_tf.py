#!/usr/bin/env python 
# to use this python script install
# pip install pyyaml

from curses.ascii import isdigit
import yaml
import rospy
import tf
import tf2_ros
import math
import numpy as np
import geometry_msgs.msg
from tf.transformations import quaternion_from_matrix, translation_from_matrix, concatenate_matrices, translation_matrix, euler_matrix

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
    odom_frame = rospy.get_param("~odom_frame")
    base_frame = rospy.get_param("~base_frame")
    camera_prefix = rospy.get_param("~prefix_camera_tf")
    num_cameras = rospy.get_param("~num_cameras")
    is_pub_cameras_tf = rospy.get_param("~publish_cameras_tf")
    is_pub_odom_tf = rospy.get_param("~publish_odom_tf")
    is_differential = rospy.get_param("~differential")
    offset_x_degree = float(rospy.get_param("~offset_x_degree"))
    offset_y_degree = float(rospy.get_param("~offset_y_degree"))
    offset_z_degree = float(rospy.get_param("~offset_z_degree"))
    camera_orientation = str(rospy.get_param("~camera_orientation"))
    I = np.identity(4)
    T_rot_offset = euler_matrix(0, math.radians(offset_y_degree), 0)
    
    T_imu_cams = []
    ## load imu_camera yaml file and read transformation cam to imu
    for i in range(num_cameras):
        T_cam_imu = getTransformation(imu_file_path, "cam"+str(i), "T_cam_imu")
        T_imu_cam = np.linalg.inv(T_cam_imu)
        T_imu_cams.append(T_imu_cam)

    # t265 camera - find the middle point of the two cameras
    # get translation vector from each camera
    t_cam1 = translation_from_matrix(T_imu_cams[0])
    t_cam2 = translation_from_matrix(T_imu_cams[1])
    # get vector between fisheye camera 1 and fisheye camera2
    t_cam1_cam2 = t_cam2 - t_cam1
    # get vector from fisheye camera 1 to the middle point of the previous vector (odometry reference)
    t_cam_odom = t_cam1_cam2 / 2
    t_cam_odom_optical = t_cam1_cam2 / 2
    # build rotation matrix to translate from camera rf to odometry rf - if camera is not use in differential mode
    # if(camera_orientation == "top" or camera_orientation == "front" and not(differential)):
    #     # align to the reference frame: xyz:        odom frame
    #     #          z                                     z y
    #     #          |__                                   |/__ 
    #     #         /    y                                      x
    #     #        x 
    #     
    # else: # bottom
    #     # align to the reference frame: xyz:
    #     #          z x
    #     #        __|/
    #     #       y
    #     
    #
    # transform from imu to camera optical frame (center of the two stereo cameras)
    T_imu_cam_optical = concatenate_matrices(translation_matrix(t_cam_odom_optical), T_imu_cams[0]) # reference frame in the optical frame
    if(is_differential):
        print("Differential ", camera_prefix)
        # the measurement will be used in differential mode
        # apply a rotation of 90 deg around x-axis followed by a rotation of 90 deg around the z-axis to transform from optical_frame to pose_frame
        T_rot = euler_matrix(0.0, -math.pi/2, math.pi/2)
        T_imu_cam_pose = concatenate_matrices(T_imu_cam_optical, T_rot) # transform between imu and camera_pose_frame
        T_imu_cam_odom = T_imu_cam_pose
    else:
        print("NOT differential ", camera_prefix)
        # build transformation to align the optical frame to the camera_odometry_frame
        T_rot = euler_matrix(math.radians(offset_x_degree), math.radians(offset_y_degree), math.radians(offset_z_degree))
        # build homogeneous transformation between IMU and the just calculated odometry reference = T_imu_camera1 * T_cam_odom * T_rot_to_odom_frame
        T_imu_cam_odom = concatenate_matrices(T_imu_cam_optical, T_rot)
        #T_imu_cam_odom = concatenate_matrices( translation_matrix(t_cam_odom), T_imu_cams[0], T_rot, T_rot_temp)
        #T_imu_cam_odom = concatenate_matrices(translation_matrix(t_cam_odom), T_imu_cams[0], T_rot_z, T_rot_y)

    ## publish transformations as TF static transfroms
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transforms = []
    # publish base_link -> imu transform - Identity
    T_base_link_imu = buildStaticTransform(I,"/base_link", "/imu")
    static_transforms.append(T_base_link_imu)
    # publish T_imu->cameras transformations
    if(is_pub_cameras_tf):
        for i,transform in enumerate(T_imu_cams):
            T_static_imu_cam = buildStaticTransform(transform,base_frame, camera_prefix + "_fisheye"+str(i))
            # stack all the transforomation in a list because broadcaster is latched to latched to /tf_static so only one stansform can be published at the time
            static_transforms.append(T_static_imu_cam)
    
    # publish T_imu->camera_frame, T_odom->camera_odometry_frame and T_camera_pose->base_link
    if(is_pub_odom_tf):
        #---- T imu -> camera_optical_frame ----#
        #T_static_imu_camera_optical = buildStaticTransform(T_imu_cam_optical, base_frame, camera_prefix + "_optical") # T imu -> camera_optical_frame
        #---- T imu -> camera_frame ----#
        T_static_imu_camera_odom = buildStaticTransform(T_imu_cam_odom, base_frame, camera_prefix + "_frame") 
        
        #---- T odom -> camera_odom_frame ----#
        T_odom_camera_odometry = T_imu_cam_odom
        #T_odom_camera_odometry[0,3] = 0
        #T_odom_camera_odometry[1,3] = 0
        #T_odom_camera_odometry[2,3] = 0
        #print("T_odom_camera_odometry",T_odom_camera_odometry)
        T_static_odom_camera_odom = buildStaticTransform(T_odom_camera_odometry, odom_frame, camera_prefix + "_odom_frame")
        
        #---- T camera_pose_frame -> base_link ----#
        if(is_differential):
            T_rot_z = euler_matrix(0.0, 0.0, math.pi/2)
            T_pose_base_link = np.linalg.inv(T_imu_cam_odom)
            T_pose_base_link = concatenate_matrices(T_pose_base_link, T_rot_z)
        else:
            T_pose_base_link = np.linalg.inv(T_imu_cam_odom)
        # T_pose_base_link[0,3] = 0
        # T_pose_base_link[1,3] = 0
        # T_pose_base_link[2,3] = 0
        T_static_pose_base_link  = buildStaticTransform(T_pose_base_link, camera_prefix + "_pose_frame", "/base_link_"+str(camera_prefix[-1]))
        # stack all the transforomation in a list
        #static_transforms.append(T_static_imu_camera_optical)
        static_transforms.append(T_static_imu_camera_odom)
        static_transforms.append(T_static_odom_camera_odom)
        static_transforms.append(T_static_pose_base_link)
        
    

    ## publish T_baselink_imu and T_odom_cam_odom transformations as TF static transfroms

    # publish all the transformations
    broadcaster.sendTransform(static_transforms)

    ## ROS spin until the end of the program
    rospy.spin()