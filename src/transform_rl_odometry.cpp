#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/Pose.h"
//#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/TwistWithCovariance.h"
#include "pose_cov_ops/pose_cov_ops.h"
#include "tf/transform_listener.h"

using namespace std;

geometry_msgs::Pose p_rotation;
ros::Publisher odom_pub;
ros::Subscriber odom_sub;
tf::Matrix3x3 matrix_rotation;
bool isDifferential;

void transformTwist(geometry_msgs::TwistWithCovariance msg_twist, geometry_msgs::TwistWithCovariance& new_twist){
    // rotate Twist - from cam#_link_frame to base_link
    tf::Vector3 linear_vel, new_linear_vel;
    tf::Vector3 new_angular_vel;    
    
    // convert Vector3 from geometry message to tf
    linear_vel.setX(msg_twist.twist.linear.x);
    linear_vel.setY(msg_twist.twist.linear.y);
    linear_vel.setZ(msg_twist.twist.linear.z);
    // apply rotation to the linear velocities
    new_linear_vel = matrix_rotation * linear_vel;
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
    newCovariance_linear_vel = matrix_rotation * matCov_linear_vel * matrix_rotation.transpose();
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




void handle_odometry_rl(const nav_msgs::Odometry::ConstPtr& msg){
    // rotate odometry if differential is set
    if(isDifferential){
        //cout << ".1 Now reveived new odometry message -> convertion in process" << endl;
        geometry_msgs::PoseWithCovariance p_rl, p_final;
        geometry_msgs::TwistWithCovariance msg_twist, new_twist, new_twist_cov;

        // get the position with covariance of the odometry message from robot_localization
        p_rl = msg->pose;
        
        //cout << "Pose rl: " << p_rl.pose << endl;
        //cout << "Rotation pose: " << p_rotation << endl;
        // Transform Pose: apply rotation to odometry of robot localization
        pose_cov_ops::compose(p_rotation, p_rl, p_final);
        if(isnan(p_final.pose.position.x) || isnan(p_final.pose.position.y) || isnan(p_final.pose.position.z) || isnan(p_final.pose.orientation.x) || isnan(p_final.pose.orientation.y) || isnan(p_final.pose.orientation.z) || isnan(p_final.pose.orientation.w) ){
            // at least one element in the transformation is nan - dropping transform
            cout << "Nan values in the transformation of the pose"<< endl;        
            return;
        }
        // get Twist with Covaraince from the message
        msg_twist = msg->twist;
        // Rotate twist to be from cam_pose to base_link
        transformTwist(msg_twist,new_twist);

        if(isnan(new_twist.twist.linear.x) || isnan(new_twist.twist.linear.y) || isnan(new_twist.twist.linear.z) ){
            // at least one element in the transformation is nan - dropping transform
            cout << "Nan values in the transformation of the twist" << endl;        
            return;
        }

        // prepare the new Odometry message
        
        //string new_frame_id =  camera + "_odom_frame";
        nav_msgs::Odometry transformed_odom = *msg;
        transformed_odom.pose = p_final; // fill in the pose with covariance the transfromed pose
        transformed_odom.twist = new_twist; // fill in the rotated twist
        //transformed_odom.twist = msg_twist; // fill in the original twist
        //cout << "transformed_odom: " << transformed_odom.pose.pose << endl;
        //cout << ".5 Now publish the converted message" << endl;
        // publish new odometry value

         //cout << "Twist rl: " << msg_twist.twist << endl;
         //cout << "Rotated twist: " << new_twist.twist << endl;
        odom_pub.publish(transformed_odom);
    }
    else{
        //No need to rotate odometry
        nav_msgs::Odometry odom = *msg;
        odom_pub.publish(odom);
    }
    

}//handle_odometry_rl


int main(int argc, char **argv) {

    // init node
    ros::init(argc, argv, "transform_odometry_from_rl");
    ros::NodeHandle n("~");
    
    string topic_pub_, topic_sub_;
    // ============== Get Input parameters: ==========================================
	n.param("differential", isDifferential, bool(false));
    n.param("odom_in_topic", topic_sub_, string("/odometry/filtered"));
	n.param("odom_out_topic", topic_pub_, string("/odometry/fused"));

    cout << "Parameters: " << endl << "diff: " << isDifferential << " odom_in: " << topic_sub_ << endl << "odom out: " << topic_pub_ << endl;
    // ============== Subscriber: ====================================================
    //odom_sub = n_.subscribe(topic_sub_, 10, &OdomTransformer::odom_listener, this);
  		
	// ============== Publishers: ====================================================
	//odom_pub = n_.advertise<nav_msgs::Odometry>(topic_pub_, 10);

    
    //init static transform
    p_rotation = geometry_msgs::Pose();

    ros::Duration(2.0).sleep();
    cout << "1. Build rotation transformation" << endl;
    try{
        ros::Time now = ros::Time::now();
        tf::Quaternion rotationQuaternion;
        rotationQuaternion.setRPY(-1.5708, 0.0, 0.0);
        rotationQuaternion =rotationQuaternion.normalize();
        cout << "rotation quaternion" << rotationQuaternion.x() << " " << rotationQuaternion.y() << " " << rotationQuaternion.z() << " " << rotationQuaternion.w() <<endl;
        matrix_rotation = tf::Matrix3x3(rotationQuaternion);
        //cout << "matrix_rotation " << matrix_rotation.c_str() << endl;
        p_rotation.orientation.x = rotationQuaternion.x();
        p_rotation.orientation.y = rotationQuaternion.y();
        p_rotation.orientation.z = rotationQuaternion.z();
        p_rotation.orientation.w = rotationQuaternion.w();

    }
    catch(tf::TransformException &ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    // subscribe and advertise topic
    cout << "2. subscribe and publish new topic" << endl;
    odom_pub  = n.advertise<nav_msgs::Odometry>(topic_pub_.c_str(), 1000);
    odom_sub = n.subscribe(topic_sub_.c_str(), 1000, handle_odometry_rl);

    ros::spin();
    return 0;

}// main