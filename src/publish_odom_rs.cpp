#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/Pose.h"
//#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/TwistWithCovariance.h"
#include "std_srvs/Empty.h"
#include "pose_cov_ops/pose_cov_ops.h"
#include "tf/transform_listener.h"

using namespace std;

geometry_msgs::Pose p_odom_camOdo, p_camPose_bl;
ros::Publisher odom_pub;
ros::Subscriber odom_sub;
string camera; // = "camera2";
string base_link; // = "base_link_2";
string prefix_odom; // = "cam2";
tf::StampedTransform t_odom_cameraOdom, t_cameraPose_bl;
geometry_msgs::PoseWithCovariance prevPose;
bool firstOdomMsg = true;
int state = 0;


int transformPose(geometry_msgs::PoseWithCovariance p_camOdo_pose, geometry_msgs::PoseWithCovariance& p_odom_bl){
    geometry_msgs::PoseWithCovariance p_odom_camPose;
    float kCovaraince, jump_threshold = 0.5;
    
    tf::Quaternion q_t265 = tf::Quaternion(p_camOdo_pose.pose.orientation.x,p_camOdo_pose.pose.orientation.y,p_camOdo_pose.pose.orientation.z,p_camOdo_pose.pose.orientation.w);
    q_t265 = q_t265.normalize();
    p_camOdo_pose.pose.orientation.x = q_t265.x();
    p_camOdo_pose.pose.orientation.y = q_t265.y();
    p_camOdo_pose.pose.orientation.z = q_t265.z();
    p_camOdo_pose.pose.orientation.w = q_t265.w();
    // Transform the odometry from odom to base_link
    // compose transformation odom <-> camera_odom_frame (+) camera_odom_frame <-> camera_pose_frame  = odom <-> camera_pose_frame
    pose_cov_ops::compose(p_odom_camOdo, p_camOdo_pose,  p_odom_camPose);
    tf::Quaternion q_temp = tf::Quaternion(p_odom_camPose.pose.orientation.x,p_odom_camPose.pose.orientation.y,p_odom_camPose.pose.orientation.z,p_odom_camPose.pose.orientation.w);
    q_temp = q_temp.normalize();
    p_odom_camPose.pose.orientation.x = q_temp.x();
    p_odom_camPose.pose.orientation.y = q_temp.y();
    p_odom_camPose.pose.orientation.z = q_temp.z();
    p_odom_camPose.pose.orientation.w = q_temp.w();
    // compose transformation odom <-> camera_pose_frame (+) camera_pose_frame <-> base_link = odom <-> base_link
    pose_cov_ops::compose(p_odom_camPose, p_camPose_bl, p_odom_bl);

    tf::Quaternion q = tf::Quaternion(p_odom_bl.pose.orientation.x,p_odom_bl.pose.orientation.y,p_odom_bl.pose.orientation.z,p_odom_bl.pose.orientation.w);
    q = q.normalize();
    p_odom_bl.pose.orientation.x = q.x();
    p_odom_bl.pose.orientation.y = q.y();
    p_odom_bl.pose.orientation.z = q.z();
    p_odom_bl.pose.orientation.w = q.w();
    kCovaraince = 10; //inflate covariance constant
    //inflate covariance of orientation
    p_odom_bl.covariance[21] = p_odom_bl.covariance[21] * kCovaraince;
    //p_odom_bl.covariance[22] = p_odom_bl.covariance[22] * kCovaraince;
    //p_odom_bl.covariance[23] = p_odom_bl.covariance[23] * kCovaraince;
    //p_odom_bl.covariance[27] = p_odom_bl.covariance[27] * kCovaraince;
    p_odom_bl.covariance[28] = p_odom_bl.covariance[28] * kCovaraince;
    //p_odom_bl.covariance[29] = p_odom_bl.covariance[29] * kCovaraince;
    //p_odom_bl.covariance[33] = p_odom_bl.covariance[33] * kCovaraince;
    //p_odom_bl.covariance[34] = p_odom_bl.covariance[34] * kCovaraince;
    p_odom_bl.covariance[35] = p_odom_bl.covariance[35] * kCovaraince;

    // check if previous messasge has more or less the same orientation of the current one - if not discard message
    if(abs(prevPose.pose.orientation.w - p_odom_bl.pose.orientation.w) > jump_threshold){
        //cout << "diff: " <<abs(prevPose.pose.orientation.w - p_odom_bl.pose.orientation.w) << endl;
        //cout << "prev pose: " << prevPose.pose.orientation.x << ", " << prevPose.pose.orientation.y << ", " << prevPose.pose.orientation.z << ", " << prevPose.pose.orientation.w << endl;
        //cout << "curr pose: " << p_odom_bl.pose.orientation.x << ", " << p_odom_bl.pose.orientation.y << ", " << p_odom_bl.pose.orientation.z << ", " << p_odom_bl.pose.orientation.w << endl;
        return -1;
    }
    else{
        prevPose = p_odom_bl;
        return 0;
    }


}//transformPose

void transformTwist(geometry_msgs::TwistWithCovariance msg_twist, geometry_msgs::TwistWithCovariance& new_twist){
    // rotate Twist - from cam#_link_frame to base_link
    tf::Vector3 linear_vel, new_linear_vel;
    tf::Vector3 new_angular_vel, angular_vel;
    // get rotation between base_link and cam_pose frame
    //tf::Matrix3x3 rot_camPose_bl = t_cameraPose_bl.getBasis().transpose(); // get rotation matrix
    tf::Matrix3x3 rot_camPose_bl = t_cameraPose_bl.getBasis().inverse(); // get rotation matrix
    tf::Vector3 trans_camPose_bl = t_cameraPose_bl.inverse().getOrigin();
    // convert Vector3 from geometry message to tf
    linear_vel.setX(msg_twist.twist.linear.x);
    linear_vel.setY(msg_twist.twist.linear.y);
    linear_vel.setZ(msg_twist.twist.linear.z);
    angular_vel.setX(msg_twist.twist.angular.x);
    angular_vel.setY(msg_twist.twist.angular.y);
    angular_vel.setZ(msg_twist.twist.angular.z);
    // apply rotation to the linear velocities
    new_linear_vel = rot_camPose_bl * linear_vel + trans_camPose_bl * rot_camPose_bl * angular_vel;
    new_angular_vel = rot_camPose_bl * angular_vel;
    
    // write back the values to the geometry message - TwistWithCovariance
    new_twist.twist.linear.x = new_linear_vel.getX();
    new_twist.twist.linear.y = new_linear_vel.getY();
    new_twist.twist.linear.z = new_linear_vel.getZ();
    new_twist.twist.angular.x = new_angular_vel.getX();
    new_twist.twist.angular.y = new_angular_vel.getY();
    new_twist.twist.angular.z = new_angular_vel.getZ();

    

    /* Rotate twist coaviance: C_new = R * C * R^T */
    tf::Matrix3x3 matCov_linear_vel, newCovariance_linear_vel;
    tf::Matrix3x3 matCov_angular_vel, newCovariance_angular_vel;

    // msg_twist.covariance is a 36 dimension vector every 6 position starts a new row of the matrix - get covariance only of linear velocity
    matCov_linear_vel.setValue( msg_twist.covariance[0], msg_twist.covariance[1], msg_twist.covariance[2],
                                msg_twist.covariance[6], msg_twist.covariance[7], msg_twist.covariance[8],
                                msg_twist.covariance[12], msg_twist.covariance[13], msg_twist.covariance[14]);
    // apply transformation C_new = Rot * Cov * Rot^T 
    newCovariance_linear_vel = rot_camPose_bl * matCov_linear_vel * rot_camPose_bl.transpose();

    // msg_twist.covariance is a 36 dimension vector every 6 position starts a new row of the matrix - get covariance only of angular velocity
    matCov_angular_vel.setValue( msg_twist.covariance[21], msg_twist.covariance[22], msg_twist.covariance[23],
                                msg_twist.covariance[27], msg_twist.covariance[28], msg_twist.covariance[29],
                                msg_twist.covariance[33], msg_twist.covariance[34], msg_twist.covariance[35]);
    // apply transformation C_new = Rot * Cov * Rot^T 
    newCovariance_angular_vel = rot_camPose_bl * matCov_angular_vel * rot_camPose_bl.transpose();
    // write back the new covariance into the geometry message - linear velocity
    new_twist.covariance[0] = newCovariance_linear_vel[0][0];
    new_twist.covariance[1] = newCovariance_linear_vel[0][1];
    new_twist.covariance[2] = newCovariance_linear_vel[0][2];
    new_twist.covariance[6] = newCovariance_linear_vel[1][0];
    new_twist.covariance[7] = newCovariance_linear_vel[1][1];
    new_twist.covariance[8] = newCovariance_linear_vel[1][2];
    new_twist.covariance[12] = newCovariance_linear_vel[2][0];
    new_twist.covariance[13] = newCovariance_linear_vel[2][1];
    new_twist.covariance[14] = newCovariance_linear_vel[2][2];
    // write back the new covariance into the geometry message - angular velocity
    new_twist.covariance[21] = newCovariance_angular_vel[0][0];
    new_twist.covariance[22] = newCovariance_angular_vel[0][1];
    new_twist.covariance[23] = newCovariance_angular_vel[0][2];
    new_twist.covariance[27] = newCovariance_angular_vel[1][0];
    new_twist.covariance[28] = newCovariance_angular_vel[1][1];
    new_twist.covariance[29] = newCovariance_angular_vel[1][2];
    new_twist.covariance[33] = newCovariance_angular_vel[2][0];
    new_twist.covariance[34] = newCovariance_angular_vel[2][1];
    new_twist.covariance[35] = newCovariance_angular_vel[2][2];

    // Visualization of the covariance
    /*cout << "covariance matrix" << endl;
    cout << newCovariance_linear_vel[0][0] << " " << newCovariance_linear_vel[0][1] << " " << newCovariance_linear_vel[0][2] << endl;
    cout << newCovariance_linear_vel[1][0] << " " << newCovariance_linear_vel[1][1] << " " << newCovariance_linear_vel[1][2] << endl;
    cout << newCovariance_linear_vel[2][0] << " " << newCovariance_linear_vel[2][1] << " " << newCovariance_linear_vel[2][2] << endl;*/

    new_twist.covariance = msg_twist.covariance;
}


void handle_odometry_rs(const nav_msgs::Odometry::ConstPtr& msg){
    //cout << ".1 Now reveived new odometry message -> convertion in process" << endl;
    geometry_msgs::PoseWithCovariance p_camOdo_pose, p_odom_bl;
    geometry_msgs::TwistWithCovariance msg_twist, new_twist, new_twist_cov;


    // get the position with covariance between camera_odom_frame and camera_pose_frame
    p_camOdo_pose = msg->pose;
    
    // Transform Pose from camera_odom <-> camera_pose to odom <-> base_link
    int resultTransformPose = transformPose(p_camOdo_pose, p_odom_bl);
    // the transformed pose has a completelly different orientation wrt to the previous one -> discard message
    // TODO investigate if this can be a numerical error when transforming the pose (the original message does not have this problem)
    if(resultTransformPose < 0){
        ROS_WARN("Jump pose detected");
        cout << "Jump pose detected at time " << msg->header.stamp << endl;     
        return;   
    }
    if(isnan(p_odom_bl.pose.position.x) || isnan(p_odom_bl.pose.position.y) || isnan(p_odom_bl.pose.position.z) || isnan(p_odom_bl.pose.orientation.x) || isnan(p_odom_bl.pose.orientation.y) || isnan(p_odom_bl.pose.orientation.z) || isnan(p_odom_bl.pose.orientation.w) ){
        // at least one element in the transformation is nan - dropping transform
        cout << "Nan values in the transformation of the pose in " << camera << endl;        
        return;
    }
    // get Twist with Covaraince from the message
    msg_twist = msg->twist;
    // Rotate twist to be from cam_pose to base_link
    transformTwist(msg_twist,new_twist);

    if(isnan(new_twist.twist.linear.x) || isnan(new_twist.twist.linear.y) || isnan(new_twist.twist.linear.z) ){
        // at least one element in the transformation is nan - dropping transform
        cout << "Nan values in the transformation of the twist in " << camera << endl;        
        return;
    }

    // prepare the new Odometry message
    //cout << ".4 Build new message" << endl;
    //string child_frame = camera + "_frame";
    string child_frame = "base_link";
    string new_frame_id;
    // everything is transformed to be in the odom frame so the frame_id of the odometry message is:
    new_frame_id =  "odom";
    //string new_frame_id =  camera + "_odom_frame";
    nav_msgs::Odometry transformed_odom = *msg;
    transformed_odom.pose = p_odom_bl; // fill in the pose with covariance the transfromed pose
    transformed_odom.twist = new_twist; // fill in the rotated twist
    //transformed_odom.twist = msg_twist; // fill in the original twist
    transformed_odom.header.frame_id = new_frame_id.c_str();    //reference frame
    transformed_odom.child_frame_id = child_frame.c_str(); //child frame

    //cout << ".5 Now publish the converted message" << endl;
    // publish new odometry value
    odom_pub.publish(transformed_odom);

    // Enable Robot Localization when at least one camera is on
    if(firstOdomMsg){
        firstOdomMsg = false;
        state = 1;
    }

}//handle_odometry_rs


int main(int argc, char **argv) {
    // init node
    ros::init(argc, argv, "republish_transformed_odom");
    ros::NodeHandle n("~");
    // init prev prose with covariance - used to filter out messages with wrong orientation wrt to the previous one
    prevPose = geometry_msgs::PoseWithCovariance();
    prevPose.pose.orientation.w = 1.0;
    
    string topic_pub_, topic_sub_;
    // ============== Get Input parameters: ==========================================
    n.param("camera", camera, string("camera")); // prefix of the camera topic for realsense t265
	n.param("base_link", base_link, string("base_link")); // base_link frame name relative to the camera#_pose_frame
    n.param("prefix_odom", prefix_odom, string("cam")); // new topic prefix name for the new odometry

    //cout << camera << " " << base_link << " " << prefix_odom << " " << endl;

    string topic_pub = "/"+prefix_odom + "/odom/sample_throttled";
    string topic_sub = "/"+camera+"/odom/sample_throttled";
    //string topic_sub = "/"+camera+"/odom/sample";
    
    //init static transform
    tf::TransformListener listener;
    //tf::StampedTransform t_odom_cameraOdom, t_cameraPose_bl;
    p_odom_camOdo = geometry_msgs::Pose();
    p_camPose_bl = geometry_msgs::Pose(); 

    ros::Duration(2.0).sleep();

    // ============== Transform odom to camera_odom_frame: ==========================================
    cout << "1. Now look up for transform from odom to camera_odom_frame" << endl;
    try{
        ros::Time now = ros::Time::now();
        listener.waitForTransform("/odom", "/"+camera+"_odom_frame", now, ros::Duration(10.0));
        listener.lookupTransform("/odom", "/"+camera+"_odom_frame", now, t_odom_cameraOdom);
        p_odom_camOdo.position.x = t_odom_cameraOdom.getOrigin().x();
        p_odom_camOdo.position.y = t_odom_cameraOdom.getOrigin().y();
        p_odom_camOdo.position.z = t_odom_cameraOdom.getOrigin().z();
        

        tf::Quaternion q_t_odom_cameraOdom = t_odom_cameraOdom.getRotation().normalize();
        p_odom_camOdo.orientation.x = q_t_odom_cameraOdom.x();
        p_odom_camOdo.orientation.y = q_t_odom_cameraOdom.y();
        p_odom_camOdo.orientation.z = q_t_odom_cameraOdom.z();
        p_odom_camOdo.orientation.w = q_t_odom_cameraOdom.w();

        //cout << "p_odo_camOdo orientation: " << p_odom_camOdo.orientation.x << " " <<p_odom_camOdo.orientation.y << " " << p_odom_camOdo.orientation.z << " " << p_odom_camOdo.orientation.w << endl;
    }
    catch(tf::TransformException &ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
    // ============== Transform camera_pose_frame to base_link: ==========================================
    cout << "2. Now look up for transform from camera_pose_frame to base_link" << endl;
    try{
        ros::Time now = ros::Time::now();
        listener.waitForTransform(("/"+camera+"_pose_frame").c_str(), base_link.c_str(), now, ros::Duration(10.0));
        listener.lookupTransform(("/"+camera+"_pose_frame").c_str(), base_link.c_str(), now, t_cameraPose_bl);
        p_camPose_bl.position.x = t_cameraPose_bl.getOrigin().x();
        p_camPose_bl.position.y = t_cameraPose_bl.getOrigin().y();
        p_camPose_bl.position.z = t_cameraPose_bl.getOrigin().z();
        
        tf::Quaternion q_t_cameraPose_bl = t_cameraPose_bl.getRotation().normalize();
        p_camPose_bl.orientation.x = q_t_cameraPose_bl.x();
        p_camPose_bl.orientation.y = q_t_cameraPose_bl.y();
        p_camPose_bl.orientation.z = q_t_cameraPose_bl.z();
        p_camPose_bl.orientation.w = q_t_cameraPose_bl.w();


    }
    catch(tf::TransformException &ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
    // ============== Subscribe and advertise topic: ==========================================
    cout << "3. Now subscribe to topic and publish new one" << endl;
    odom_pub  = n.advertise<nav_msgs::Odometry>(topic_pub.c_str(), 100);
    odom_sub = n.subscribe(topic_sub.c_str(), 100, handle_odometry_rs);

    ros::Rate r(20); // 20 hz
    while (state == 0){ // still haven't received one odom message yet
        ros::spinOnce();
        r.sleep();
    }

    // now I am sure that at least one camera is up - enable robot_localization
    //call service /ekf_odom/enable
    ros::service::waitForService("/ekf_odom/enable");
    ros::ServiceClient startRL = n.serviceClient<std_srvs::Empty>("/ekf_odom/enable");
    std_srvs::Empty srv;
    startRL.call(srv);
    ros::spin();
    return 0;

}// main