import rosbag
import rospy
import os
from shutil import move

bag_location ="/home/giacomo/catkin_ws/bags/inspectrone/2022-02-02-14-08-19.bag"

orig = os.path.splitext(bag_location)[0] + ".orig.bag"
first = True
delta_vect = [0.0, 0.0, 0.0] # dx, dy, dz
move(bag_location, orig)

with rosbag.Bag(bag_location, 'w') as outbag:
    for topic, msg, t in rosbag.Bag(orig).read_messages(): # topic, message, time
        if(topic == "/camera/odom/sample" or topic == "/camera/odom/sample_throttled"):
            if(first):
                delta_vect[0] = msg.pose.pose.position.x
                delta_vect[1] = msg.pose.pose.position.y
                delta_vect[2] = msg.pose.pose.position.z
                first = False
            else:
                msg.pose.pose.position.x -= delta_vect[0]
                msg.pose.pose.position.y -= delta_vect[1]
                msg.pose.pose.position.z -= delta_vect[2]
                outbag.write(topic, msg,t)
        elif (topic=="/tf"):
            if(msg.transforms[0].header.frame_id == "camera_odom_frame" and msg.transforms[0].child_frame_id == "camera_pose_frame"):
                if(first):
                    delta_vect[0] = msg.transforms[0].transform.translation.x
                    delta_vect[1] = msg.transforms[0].transform.translation.y
                    delta_vect[2] = msg.transforms[0].transform.translation.z
                    first = False
                else:
                    msg.transforms[0].transform.translation.x -= delta_vect[0]
                    msg.transforms[0].transform.translation.y -= delta_vect[1]
                    msg.transforms[0].transform.translation.z -= delta_vect[2]
                    outbag.write(topic, msg,t)
            else:
                outbag.write(topic, msg,t)


        else: # all the other topics
            outbag.write(topic, msg,t)

    print("End of the program")