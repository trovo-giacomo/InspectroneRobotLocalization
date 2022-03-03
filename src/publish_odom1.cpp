#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/Pose.h"
#include "pose_cov_ops/pose_cov_ops.h"
#include "tf/transform_listener.h"

using namespace std;

geometry_msgs::Pose p_odom_camOdo, p_camPose_bl;
ros::Publisher odom_pub;
ros::Subscriber odom_sub;

string camera = "camera1";
string base_link = "base_link_1";
string prefix_odom = "cam1";
string node_name = "publish_odom1";


void handle_odometry_rs(const nav_msgs::Odometry::ConstPtr& msg){
    cout << ".1 Now reveived new odometry message -> convertion in process" << endl;
    geometry_msgs::PoseWithCovariance p_camOdo_pose, p_odom_camPose, p_odom_bl;
    // get the position with covariance between camera_odom_frame and camera_pose_frame
    p_camOdo_pose = msg->pose;

    /// Transformation
    cout << ".2 Transform from odom to camera pose frame" << endl;


    // compose transformation odom <-> camera_odom_frame (+) camera_odom_frame <-> camera_pose_frame  = odom <-> camera_pose_frame
    pose_cov_ops::compose(p_odom_camOdo, p_camOdo_pose,  p_odom_camPose);
    cout << ".3 Transform from odom to base_link" << endl;
    // compose transformation odom <-> camera_pose_frame (+) camera_pose_frame <-> base_link = odom <-> base_link
    pose_cov_ops::compose(p_odom_camPose, p_camPose_bl, p_odom_bl);

    // prepare the new Odometry message
    cout << ".4 Build new message" << endl;
    //string child_frame = prefix_odom + "_link_frame";
    string child_frame = "base_link";
    nav_msgs::Odometry transformed_odom = *msg;
    transformed_odom.pose = p_odom_bl; // fill in the pose with covariance the transfromed pose
    transformed_odom.header.frame_id = "odom";    //reference frame
    transformed_odom.child_frame_id = child_frame.c_str(); //child frame

    cout << ".5 Now publish the converted message" << endl;
    // publish new odometry value
    odom_pub.publish(transformed_odom);

}//handle_odometry_rs


int main(int argc, char **argv) {
    // init node
    ros::init(argc, argv, node_name.c_str());
    ros::NodeHandle n;
    string topic_pub = "/"+prefix_odom + "/odom/sample_throttled";
    string topic_sub = "/"+camera+"/odom/sample_throttled";
    

    //init static transform
    tf::TransformListener listener;
    tf::StampedTransform t_odom_cameraOdom, t_cameraPose_bl;
    p_odom_camOdo = geometry_msgs::Pose();
    p_camPose_bl = geometry_msgs::Pose(); 

    ros::Duration(2.0).sleep();
    cout << "1. Now look up for transform from odom to camera_odom_frame" << endl;
    try{
        ros::Time now = ros::Time::now();
        listener.waitForTransform("/odom", "/"+camera+"_odom_frame", now, ros::Duration(10.0));
        listener.lookupTransform("/odom", "/"+camera+"_odom_frame", now, t_odom_cameraOdom);
        p_odom_camOdo.position.x = t_odom_cameraOdom.getOrigin().x();
        p_odom_camOdo.position.y = t_odom_cameraOdom.getOrigin().y();
        p_odom_camOdo.position.z = t_odom_cameraOdom.getOrigin().z();
        

        float s = t_cameraPose_bl.getRotation().getW();
        p_odom_camOdo.orientation.x = t_odom_cameraOdom.getRotation().x();
        p_odom_camOdo.orientation.y = t_odom_cameraOdom.getRotation().y();
        p_odom_camOdo.orientation.z = t_odom_cameraOdom.getRotation().z();
        p_odom_camOdo.orientation.w = t_odom_cameraOdom.getRotation().w();

        //cout << "p_odo_camOdo orientation: " << p_odom_camOdo.orientation.x << " " <<p_odom_camOdo.orientation.y << " " << p_odom_camOdo.orientation.z << " " << p_odom_camOdo.orientation.w << endl;
    }
    catch(tf::TransformException &ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    cout << "2. Now look up for transform from camera_pose_frame to base_link" << endl;
    try{
        ros::Time now = ros::Time::now();
        listener.waitForTransform(("/"+camera+"_pose_frame").c_str(), base_link.c_str(), now, ros::Duration(10.0));
        listener.lookupTransform(("/"+camera+"_pose_frame").c_str(), base_link.c_str(), now, t_cameraPose_bl);
        p_camPose_bl.position.x = t_cameraPose_bl.getOrigin().x();
        p_camPose_bl.position.y = t_cameraPose_bl.getOrigin().y();
        p_camPose_bl.position.z = t_cameraPose_bl.getOrigin().z();
        
        float s = t_cameraPose_bl.getRotation().getW();
        p_camPose_bl.orientation.x = t_cameraPose_bl.getRotation().x();
        p_camPose_bl.orientation.y = t_cameraPose_bl.getRotation().y();
        p_camPose_bl.orientation.z = t_cameraPose_bl.getRotation().z();
        p_camPose_bl.orientation.w = t_cameraPose_bl.getRotation().w();
        //cout << "p_camPose_bl orientation: " << p_camPose_bl.orientation.x << " " <<p_camPose_bl.orientation.y << " " << p_camPose_bl.orientation.z << " " << p_camPose_bl.orientation.z << endl;


    }
    catch(tf::TransformException &ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    // subscribe and advertise topic
    cout << "3. Now subscribe to topic and publish new one" << endl;
    odom_pub  = n.advertise<nav_msgs::Odometry>(topic_pub.c_str(), 1000);
    odom_sub = n.subscribe(topic_sub.c_str(), 1000, handle_odometry_rs);

    ros::spin();

    /*
    ros::Rate loopRate(10);
    while(ros::ok()){
        cout << "Publishing transfomed odometry from " << camera << " to topic " << prefix_odom << "/odom/sample_throtteld";
        ros::spinOnce();
        loopRate.sleep();
    }*/

    return 0;

}// main