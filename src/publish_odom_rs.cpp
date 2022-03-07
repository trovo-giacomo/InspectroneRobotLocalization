#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/Pose.h"
//#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/TwistWithCovariance.h"
#include "pose_cov_ops/pose_cov_ops.h"
#include "tf/transform_listener.h"

using namespace std;

geometry_msgs::Pose p_odom_camOdo, p_camPose_bl;
ros::Publisher odom_pub;
ros::Subscriber odom_sub;
string camera; // = "camera2";
string base_link; // = "base_link_2";
string prefix_odom; // = "cam2";
string node_name; // = "publish_odom2";
tf::StampedTransform t_odom_cameraOdom, t_cameraPose_bl;



void transformPose(geometry_msgs::PoseWithCovariance p_camOdo_pose, geometry_msgs::PoseWithCovariance& p_odom_bl){
    geometry_msgs::PoseWithCovariance p_odom_camPose;
    //cout << ".2 Transform from odom to camera pose frame" << endl;
    // compose transformation odom <-> camera_odom_frame (+) camera_odom_frame <-> camera_pose_frame  = odom <-> camera_pose_frame
    pose_cov_ops::compose(p_odom_camOdo, p_camOdo_pose,  p_odom_camPose);
    //cout << ".3 Transform from odom to base_link" << endl;
    // compose transformation odom <-> camera_pose_frame (+) camera_pose_frame <-> base_link = odom <-> base_link
    pose_cov_ops::compose(p_odom_camPose, p_camPose_bl, p_odom_bl);
    
}//transformPose

void transformTwist(geometry_msgs::TwistWithCovariance msg_twist, geometry_msgs::TwistWithCovariance& new_twist){
    // rotate Twist - from cam#_link_frame to base_link
    tf::Vector3 linear_vel, new_linear_vel;
    tf::Vector3 new_angular_vel;
    // get rotation between base_link and cam_pose frame
    tf::Matrix3x3 rot_camPose_bl = t_cameraPose_bl.getBasis().transpose(); // get rotation matrix
    // convert Vector3 from geometry message to tf
    linear_vel.setX(msg_twist.twist.linear.x);
    linear_vel.setY(msg_twist.twist.linear.y);
    linear_vel.setZ(msg_twist.twist.linear.z);
    // apply rotation to the linear velocities
    new_linear_vel = rot_camPose_bl * linear_vel;
    //cout << "linear vel" << " " <<  linear_vel.getX() << " " <<  linear_vel.getY() << " " <<  linear_vel.getZ() << endl;
    //cout << "rot matrix" << rot_camPose_bl.str() << endl;
    //cout << "result vel" << " "<< new_linear_vel.getX() << " "<< new_linear_vel.getY() << " "<< new_linear_vel.getZ() << endl;
    
    // write back the values to the geometry message - TwistWithCovariance
    new_twist.twist.linear.x = new_linear_vel.getX();
    new_twist.twist.linear.y = new_linear_vel.getY();
    new_twist.twist.linear.z = new_linear_vel.getZ();

    // rotate the covariance matrix
    tf::Matrix3x3 matCov_linear_vel, newCovariance_linear_vel;

    // msg_twist.covariance is a 36 dimension vector every 6 position starts a new row of the matrix - get covariance only of linear velocity
    matCov_linear_vel.setValue( msg_twist.covariance[0], msg_twist.covariance[1], msg_twist.covariance[2],
                                msg_twist.covariance[6], msg_twist.covariance[7], msg_twist.covariance[8],
                                msg_twist.covariance[12], msg_twist.covariance[13], msg_twist.covariance[14]);
    // apply transformation C_new = Rot * Cov * Rot^T 
    newCovariance_linear_vel = rot_camPose_bl * matCov_linear_vel * rot_camPose_bl.transpose();
    // write back the new covariance into the geometry message
    new_twist.covariance[0] = newCovariance_linear_vel[0][0];
    new_twist.covariance[1] = newCovariance_linear_vel[0][1];
    new_twist.covariance[2] = newCovariance_linear_vel[0][2];
    new_twist.covariance[6] = newCovariance_linear_vel[1][0];
    new_twist.covariance[7] = newCovariance_linear_vel[1][1];
    new_twist.covariance[8] = newCovariance_linear_vel[1][2];
    new_twist.covariance[12] = newCovariance_linear_vel[2][0];
    new_twist.covariance[13] = newCovariance_linear_vel[2][1];
    new_twist.covariance[14] = newCovariance_linear_vel[2][2];
    
    // Visualization of the covariance
    /*cout << "covariance matrix" << endl;
    cout << newCovariance_linear_vel[0][0] << " " << newCovariance_linear_vel[0][1] << " " << newCovariance_linear_vel[0][2] << endl;
    cout << newCovariance_linear_vel[1][0] << " " << newCovariance_linear_vel[1][1] << " " << newCovariance_linear_vel[1][2] << endl;
    cout << newCovariance_linear_vel[2][0] << " " << newCovariance_linear_vel[2][1] << " " << newCovariance_linear_vel[2][2] << endl;*/
}



void handle_odometry_rs(const nav_msgs::Odometry::ConstPtr& msg){
    //cout << ".1 Now reveived new odometry message -> convertion in process" << endl;
    geometry_msgs::PoseWithCovariance p_camOdo_pose, p_odom_bl;
    geometry_msgs::TwistWithCovariance msg_twist, new_twist;

    // get the position with covariance between camera_odom_frame and camera_pose_frame
    p_camOdo_pose = msg->pose;
    
    // Transform Pose from camera_odom <-> camera_pose to odom <-> base_link
    transformPose(p_camOdo_pose, p_odom_bl);
    // get Twist with Covaraince from the message
    msg_twist = msg->twist;
    // Rotate twist to be from cam_pose to base_link
    transformTwist(msg_twist,new_twist);

    // prepare the new Odometry message
    //cout << ".4 Build new message" << endl;
    //string child_frame = prefix_odom + "_link_frame";
    string child_frame = "base_link";
    nav_msgs::Odometry transformed_odom = *msg;
    transformed_odom.pose = p_odom_bl; // fill in the pose with covariance the transfromed pose
    transformed_odom.twist = new_twist; // fill in the rotated twist
    transformed_odom.header.frame_id = "odom";    //reference frame
    transformed_odom.child_frame_id = child_frame.c_str(); //child frame

    //cout << ".5 Now publish the converted message" << endl;
    // publish new odometry value
    odom_pub.publish(transformed_odom);

}//handle_odometry_rs


int main(int argc, char **argv) {
    if(argc != 7){
        //cout << "The node need to be called with 4 arguments" << endl;
        //cout << "Called with " << argc << " parameters" << endl;
        //for(int i=0; i<argc; i++){
        //   cout << "param" << i <<": " << argv[i] << endl;
        //}//for
        cout << "Not enough arguments" << endl;
        return -1;
    }
    else{
        camera = argv[1]; // prefix of the camera topic for realsense t265
        base_link = argv[2]; //base_link frame name relative to the camera#_pose_frame
        prefix_odom = argv[3]; //new topic prefix name for the new odometry
        node_name = argv[4]; //name of the node
        cout << camera << " " << base_link << " " << prefix_odom << " " << node_name << endl;
    }

    // init node
    ros::init(argc, argv, node_name.c_str());
    ros::NodeHandle n;
    string topic_pub = "/"+prefix_odom + "/odom/sample_throttled";
    string topic_sub = "/"+camera+"/odom/sample_throttled";
    
    //init static transform
    tf::TransformListener listener;
    //tf::StampedTransform t_odom_cameraOdom, t_cameraPose_bl;
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
    return 0;

}// main