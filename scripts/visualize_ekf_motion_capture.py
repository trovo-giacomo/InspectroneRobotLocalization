#!/usr/bin/env python
###################################
####  Plot EKF with Optitrack ####
###################################

# Author: Giacomo Trovo (s202447)
# Master thesis

import pandas as pd
import matplotlib.pyplot as plt
import glob




if __name__ == '__main__':
  path = "/home/giacomo/inspectrone_ws/bags/localization/fixed_orientation/results_v2_2/"
  bagName="bag_position_v2_2"
  file_names = glob.glob(path+bagName+"*.csv")
  file_names.sort()
  #print(file_names)

  # for each bag load the 5 files: *_cam1.csv, *_cam2.csv, *_cam3.csv, *_ekf.csv, *_optitrack.csv
  for i in range(0,len(file_names),5):
    cam1_file = file_names[i]
    cam2_file = file_names[i+1]
    cam3_file = file_names[i+2]
    ekf_file = file_names[i+3]
    optitrack_file = file_names[i+4]
    
    # read data
    cam1_bag = pd.read_csv(cam1_file)
    cam2_bag = pd.read_csv(cam2_file)
    cam3_bag = pd.read_csv(cam3_file)
    ekf_bag = pd.read_csv(ekf_file)
    optitrack_bag = pd.read_csv(optitrack_file)
    # plot x, y, z camera odometry vs ekf estimate vs motion capture
    #print("Plot the three path together - odometry, GPS and estimate")
    
    fig, ax = plt.subplots(1,3, sharex=True, figsize=(20,20))
    #plt.title("Estimate position")
    #plt.plot(gps_data['GPS_x'], gps_data['GPS_y'],'o') # GPS data not rotated
    #ax[0].plot(cam1_bag['field.header.stamp'], cam1_bag['field.pose.pose.position.x'])
    #ax[0].plot(cam2_bag['field.header.stamp'], cam2_bag['field.pose.pose.position.x'])
    #ax[0].plot(cam3_bag['field.header.stamp'], cam3_bag['field.pose.pose.position.x'])
    ax[0].plot(ekf_bag['field.header.stamp'], ekf_bag['field.pose.pose.position.x'])
    ax[0].plot(optitrack_bag['field.header.stamp'], optitrack_bag['field.pose.position.x'])
    ax[0].grid()
    ax[0].set_xlabel('Time')
    ax[0].set_ylabel('X')
    #ax[0].legend(["Camera 1", "Camera 2", "Camera 3", "RL", "Optitrack"])
    ax[0].legend(["RL", "Optitrack"])

    #ax[1].plot(cam1_bag['field.header.stamp'], cam1_bag['field.pose.pose.position.y'])
    #ax[1].plot(cam2_bag['field.header.stamp'], cam2_bag['field.pose.pose.position.y'])
    #ax[1].plot(cam3_bag['field.header.stamp'], cam3_bag['field.pose.pose.position.y'])
    ax[1].plot(ekf_bag['field.header.stamp'], ekf_bag['field.pose.pose.position.y'])
    ax[1].plot(optitrack_bag['field.header.stamp'], optitrack_bag['field.pose.position.y'])
    ax[1].grid()
    ax[1].set_xlabel('Time')
    ax[1].set_ylabel('Y')
    #ax[1].legend(["Camera 1", "Camera 2", "Camera 3", "RL", "Optitrack"])
    ax[1].legend(["RL", "Optitrack"])

    #ax[2].plot(cam1_bag['field.header.stamp'], cam1_bag['field.pose.pose.position.z'])
    #ax[2].plot(cam2_bag['field.header.stamp'], cam2_bag['field.pose.pose.position.z'])
    #ax[2].plot(cam3_bag['field.header.stamp'], cam3_bag['field.pose.pose.position.z'])
    ax[2].plot(ekf_bag['field.header.stamp'], ekf_bag['field.pose.pose.position.z'])
    ax[2].plot(optitrack_bag['field.header.stamp'], optitrack_bag['field.pose.position.z'])
    ax[2].grid()
    ax[2].set_xlabel('Time')
    ax[2].set_ylabel('Z')
    #ax[2].legend(["Camera 1", "Camera 2", "Camera 3", "RL", "Optitrack"])
    ax[2].legend(["RL", "Optitrack"])


    fig.suptitle(file_names[i].split("/")[-1].split(".")[0].split("_cam1")[0])
    plt.show()