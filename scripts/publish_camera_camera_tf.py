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

if __name__ == '__main__':
        
    file_path = rospy.get_param("yaml_file_path")
    reference_frame = rospy.get_param("reference_frame")
    child_frame = rospy.get_param("child_frame")
    num_cameras = rospy.get_param("num_cameras")
    print("FILE PATH",file_path)
    rospy.init_node("publish_camera_tf_from_yaml")
    static_transforms = []
    I = np.identity(4)
    T_all = [I]
    with open(file_path, 'r') as config_file:
        try:
            #for i in range(1,6):
            rs_camera_intr_extr_params = yaml.safe_load_all(config_file)
            for params in rs_camera_intr_extr_params:
                for i in range(1,num_cameras):
                    camera = "cam"+str(i)
                
                    #print(params)
                    cam = params[camera]
                    print(camera)
                    T_cam_cam = cam["T_cn_cnm1"]
                    # concatenate the transformations that come before the original camera
                    T_cam_cam0 = concatenate_matrices(T_cam_cam, T_all[-1])
                    T_all.append(T_cam_cam0)
                                         
                    # rotation matrix
                    # T_cam0_imu[0][0], T_cam0_imu[0][1], T_cam0_imu[0][2]
                    # T_cam0_imu[1][0], T_cam0_imu[1][1], T_cam0_imu[1][2]
                    # T_cam0_imu[2][0], T_cam0_imu[2][1], T_cam0_imu[2][2]
                    quat_cam_imu = quaternion_from_matrix(T_cam_cam0)
                    # translation vector
                    tran_cam_imu = translation_from_matrix(T_cam_cam0)
                    # Send static trnasform cam0 imu
                    broadcaster = tf2_ros.StaticTransformBroadcaster()
                    
                    static_transform_cam_imu = geometry_msgs.msg.TransformStamped()

                    static_transform_cam_imu.header.stamp = rospy.Time.now()
                    static_transform_cam_imu.header.frame_id = reference_frame
                    static_transform_cam_imu.child_frame_id = child_frame + "_"+str(i)

                    static_transform_cam_imu.transform.translation.x = tran_cam_imu[0]
                    static_transform_cam_imu.transform.translation.y = tran_cam_imu[1]
                    static_transform_cam_imu.transform.translation.z = tran_cam_imu[2]

                    static_transform_cam_imu.transform.rotation.x = quat_cam_imu[0]
                    static_transform_cam_imu.transform.rotation.y = quat_cam_imu[1]
                    static_transform_cam_imu.transform.rotation.z = quat_cam_imu[2]
                    static_transform_cam_imu.transform.rotation.w = quat_cam_imu[3]
                    # stack all the rtansforomation in a list because broadcaster is latched to latched to /tf_static so only one stansform can be published at the time
                    static_transforms.append(static_transform_cam_imu)
                    

        except yaml.YAMLError as exc:
            print(exc)

        broadcaster.sendTransform(static_transforms)
        rospy.spin()
    