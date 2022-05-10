#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/Pose.h"
//#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/TwistWithCovariance.h"
#include "pose_cov_ops/pose_cov_ops.h"
#include "tf/transform_listener.h"

using namespace std;

void transformTwist(){
    

    

    /* Rotate twist coaviance: C_new = R * C * R^T */
    tf::Matrix3x3 matCov_linear_vel, newCovariance_linear_vel, rot_camPose_bl;
    tf::Matrix3x3 matCov_angular_vel, newCovariance_angular_vel;
    // rotate around z-axis by 90 deg
    rot_camPose_bl.setValue(0, -1, 0,
                            1,  0, 0,
                            0,  0, 1);

    // msg_twist.covariance is a 36 dimension vector every 6 position starts a new row of the matrix - get covariance only of linear velocity
    matCov_linear_vel.setValue( 5.0, 0.0, 0.0,
                                0.0, 2.0, 0.0,
                                0.0, 0.0, 3.0);
    
    // apply transformation C_new = Rot * Cov * Rot^T 
    newCovariance_linear_vel = rot_camPose_bl * matCov_linear_vel * rot_camPose_bl.transpose();

    // msg_twist.covariance is a 36 dimension vector every 6 position starts a new row of the matrix - get covariance only of angular velocity
    matCov_angular_vel.setValue(1.0, 2.0, 3.0,
                                4.0, 5.0, 6.0,
                                7.0, 8.0, 9.0);
    // apply transformation C_new = Rot * Cov * Rot^T 
    newCovariance_angular_vel = rot_camPose_bl * matCov_angular_vel * rot_camPose_bl.transpose();


    // Visualization of the covariance
    cout << "covariance matrix - linear velocity" << endl;
    cout << matCov_linear_vel[0][0] << " " << matCov_linear_vel[0][1] << " " << matCov_linear_vel[0][2] << endl;
    cout << matCov_linear_vel[1][0] << " " << matCov_linear_vel[1][1] << " " << matCov_linear_vel[1][2] << endl;
    cout << matCov_linear_vel[2][0] << " " << matCov_linear_vel[2][1] << " " << matCov_linear_vel[2][2] << endl;
    cout << "NEW covariance matrix - linear velocity" << endl;
    cout << newCovariance_linear_vel[0][0] << " " << newCovariance_linear_vel[0][1] << " " << newCovariance_linear_vel[0][2] << endl;
    cout << newCovariance_linear_vel[1][0] << " " << newCovariance_linear_vel[1][1] << " " << newCovariance_linear_vel[1][2] << endl;
    cout << newCovariance_linear_vel[2][0] << " " << newCovariance_linear_vel[2][1] << " " << newCovariance_linear_vel[2][2] << endl;

    cout << "covariance matrix - aungular velocity" << endl;
    cout << matCov_angular_vel[0][0] << " " << matCov_angular_vel[0][1] << " " << matCov_angular_vel[0][2] << endl;
    cout << matCov_angular_vel[1][0] << " " << matCov_angular_vel[1][1] << " " << matCov_angular_vel[1][2] << endl;
    cout << matCov_angular_vel[2][0] << " " << matCov_angular_vel[2][1] << " " << matCov_angular_vel[2][2] << endl;
    cout << "NEW covariance matrix - angular velocity" << endl;
    cout << newCovariance_angular_vel[0][0] << " " << newCovariance_angular_vel[0][1] << " " << newCovariance_angular_vel[0][2] << endl;
    cout << newCovariance_angular_vel[1][0] << " " << newCovariance_angular_vel[1][1] << " " << newCovariance_angular_vel[1][2] << endl;
    cout << newCovariance_angular_vel[2][0] << " " << newCovariance_angular_vel[2][1] << " " << newCovariance_angular_vel[2][2] << endl;

}

int main(int argc, char **argv) {
    // init node
    ros::init(argc, argv, "republish_transformed_odom");
    ros::NodeHandle n("~");
    
    transformTwist();

    //ros::spin();
    return 0;

}// main