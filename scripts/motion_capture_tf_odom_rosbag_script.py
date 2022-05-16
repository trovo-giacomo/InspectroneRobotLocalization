from turtle import position
import rosbag
import os
import math
import sys
from tf.transformations import quaternion_matrix, quaternion_multiply, quaternion_inverse, concatenate_matrices, translation_matrix, quaternion_conjugate, quaternion_from_euler, euler_from_quaternion
import geometry_msgs.msg
import numpy as np
from shutil import move

bag_location ="/home/giacomo/inspectrone_ws/bags/localization/waypoint_motion_capture/waypoint_square_v1_10.bag"

# translate and rotate motion capture pose of drone_2 in the same reference frame as the odometry message
# 1. wait the first message from both topic and get both orientations
# 2. find rotation around z axis between the two reference frames and relative offset (position of the drone in the motion capture)
# 3. rotate and translate the position of the motion capture to be alligned to the odometry

orig = os.path.splitext(bag_location)[0] + ".orig.bag"
first_odometry = True
first_motion_capture = True
received_both_orientations = False
delta_vect = np.array([0.0, 0.0, 0.0]) # dx, dy, dz
quaternion_odom = np.array([0.0, 0.0, 0.0, 0.0])
quaternion_motion_capture = np.array([0.0, 0.0, 0.0, 0.0])

if(os.path.exists(orig)):
    print("--- Origin file already exists ---")
    print("--- Removing previous file: "+str(bag_location)+" ---")
    os.remove(bag_location)
    print("--- Renaming original bag ---")
    os.rename(orig,bag_location)
    #sys.exit("System exiting - Origin bag file already exists")

print("--- Backup original bag in "+ str(bag_location.split("/")[-1].split(".")[0])+".orig.bag ---")
move(bag_location, orig)

print("--- Start processing the bag ---")
with rosbag.Bag(bag_location, 'w') as outbag:
    for topic, msg, t in rosbag.Bag(orig).read_messages(): # topic, message, time
        if(topic == "/odometry/filtered"):
            if(first_odometry):
                quaternion_odom[0] =  msg.pose.pose.orientation.x
                quaternion_odom[1] =  msg.pose.pose.orientation.y
                quaternion_odom[2] =  msg.pose.pose.orientation.z
                quaternion_odom[3] =  msg.pose.pose.orientation.w

                first_odometry = False            
        elif (topic =="/drone_2/pose"):
            if(first_motion_capture):
                # get quaternion in the map frame
                quaternion_motion_capture[0] =  msg.pose.orientation.x
                quaternion_motion_capture[1] =  msg.pose.orientation.y
                quaternion_motion_capture[2] =  msg.pose.orientation.z
                quaternion_motion_capture[3] =  msg.pose.orientation.w
                # get initial position in the map frame
                delta_vect[0] = msg.pose.position.x
                delta_vect[1] = msg.pose.position.y
                delta_vect[2] = msg.pose.position.z

                first_motion_capture = False

        if((not first_motion_capture) and (not first_odometry) and topic=="/drone_2/pose"):
            #print("t: "+str(t)+" rotating pose...")
            #print("from ")
            # I got both orientaitons (quatenrions) now rotate motion capture pose
            # 2. find rotation around z axis between the two reference frames and relative offset (position of the drone in the motion capture)
            q_relative_manual = quaternion_from_euler(0.0, 0.0, -math.pi/2)
            q_odom_capture = quaternion_multiply(q_relative_manual, quaternion_motion_capture)
            q_relative = quaternion_multiply(quaternion_odom, quaternion_inverse(q_odom_capture))
            #print("quaternion_odom",quaternion_odom)
            #print("quaternion_motion_capture",quaternion_motion_capture)
            #print("q_relative_quat_ops", q_relative_quat_ops)
            #print("q_relative_manual", q_relative_manual)
            #print("----------------------------------------")
            q_relative = q_relative_manual
            #q_relative = quaternion_motion_capture

            #T_rot_motion_capture_odom = quaternion_matrix(diff_quaternions)
            #T_rot_motion_capture = quaternion_matrix(quaternion_motion_capture)
            #T_rot_odom = quaternion_matrix(quaternion_odom)
            #T_translation_motion_capture_odom = translation_matrix(delta_vect)

            #T_motion_capture_odom = concatenate_matrices(T_rot_motion_capture_odom,T_translation_motion_capture_odom)

            #T_transformed_pose =  T_motion_capture_odom * position
            #new_position = translation_from_matrix(T_transformed_pose)
            #new_quaternion = quaternion_from_matrix(T_transformed_pose)

            # 3. rotate and translate the position of the motion capture to be alligned to the odometry
            q_temp = np.array([0.0, 0.0, 0.0, 0.0])
            q_temp[0] =  msg.pose.orientation.x 
            q_temp[1] =  msg.pose.orientation.y
            q_temp[2] =  msg.pose.orientation.z
            q_temp[3] =  msg.pose.orientation.w

            # new quaternion orientation
            new_quaternion = quaternion_multiply(q_relative, q_temp)
            # new vector rotated by q_relative
            position_temp = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, 1])
            # q2 = list(position_temp)
            # q2.append(0.0)
            # new_position = quaternion_multiply(
            #     quaternion_multiply(q_relative, q2), 
            #     quaternion_conjugate(q_relative)
            #     )[:3]
            T_rot = quaternion_matrix(q_relative)
            #print(T_rot)
            T_trans = translation_matrix(-delta_vect)
            T_final = concatenate_matrices(T_rot, T_trans)
            new_position = np.matmul(T_final, position_temp)



            #new_pose = geometry_msgs.msg.PoseStamped()            
            #new_pose.header = msg.header
            msg.header.frame_id = "odom"
            msg.pose.position.x = new_position[0]
            msg.pose.position.y = new_position[1]
            msg.pose.position.z = new_position[2]
            msg.pose.orientation.x = new_quaternion[0]
            msg.pose.orientation.y = new_quaternion[1]
            msg.pose.orientation.z = new_quaternion[2]
            msg.pose.orientation.w = new_quaternion[3]

            # 4. write it into the bag
            outbag.write(topic, msg,t)

        # elif (topic=="/tf"):
        #     if(msg.transforms[0].header.frame_id == "camera_odom_frame" and msg.transforms[0].child_frame_id == "camera_pose_frame"):
        #         if(first_odometry):
        #             delta_vect[0] = msg.transforms[0].transform.translation.x
        #             delta_vect[1] = msg.transforms[0].transform.translation.y
        #             delta_vect[2] = msg.transforms[0].transform.translation.z
        #             first_odometry = False
        #         else:
        #             msg.transforms[0].transform.translation.x -= delta_vect[0]
        #             msg.transforms[0].transform.translation.y -= delta_vect[1]
        #             msg.transforms[0].transform.translation.z -= delta_vect[2]
        #             outbag.write(topic, msg,t)
        #    else:
        #        outbag.write(topic, msg,t)


        else: # all the other topics
            outbag.write(topic, msg,t)

    print("--- End of the program ---")