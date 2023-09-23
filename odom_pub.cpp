/*
 * Automatic Addison
 * Date: May 20, 2021
 * ROS Version: ROS 1 - Melodic
 * Website: https://automaticaddison.com
 * Publishes odometry information for use with robot_pose_ekf package.
 *   This odometry information is based on wheel encoder tick counts.
 * Subscribe: ROS node that subscribes to the following topics:
 *  right_ticks : Tick counts from the right motor encoder (std_msgs/Int16)
 * 
 *  left_ticks : Tick counts from the left motor encoder  (std_msgs/Int16)
 * 
 *  initial_2d : The initial position and orientation of the robot.
 *               (geometry_msgs/PoseStamped)
 *
 * Publish: This node will publish to the following topics:
 *  odom_data_euler : Position and velocity estimate. The orientation.z 
 *                    variable is an Euler angle representing the yaw angle.
 *                    (nav_msgs/Odometry)
 *  odom_data_quat : Position and velocity estimate. The orientation is 
 *                   in quaternion format.
 *                   (nav_msgs/Odometry)
 * Modified from Practical Robotics in C++ book (ISBN-10 : 9389423465)
 *   by Lloyd Brombach
 */
 
// Include various libraries
#include "ros/ros.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include "std_msgs/Int16.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>


// Create odometry data publishers
ros::Publisher odom_data_euler_pub;
ros::Publisher odom_data_pub_quat;
nav_msgs::Odometry odomNew;
nav_msgs::Odometry odomOld;

// Initial pose
const double initialX = 0.0;
const double initialY = 0.0;
const double initialTheta = 0.00000000001;
const double PI = 3.141592;

// Robot physical constants
const double TICKS_PER_REVOLUTION = 12*210.59; // For reference purposes.
const double WHEEL_RADIUS = 0.060; // Wheel radius in meters
const double WHEEL_BASE = 0.205; // Center of left tire to center of right tire
const double TICKS_PER_METER = TICKS_PER_REVOLUTION*(1/(WHEEL_RADIUS*2*PI)); // Original was 2800

// Distance both wheels have traveled
double distanceLeft = 0;
double distanceRight = 0;

// Flag to see if initial pose has been received
bool initialPoseRecieved = false;

using namespace std;
 
// Get initial_2d message from either Rviz clicks or a manual pose publisher
void set_initial_2d(const geometry_msgs::PoseStamped &rvizClick) {
  odomOld.pose.pose.position.x = 0;
  odomOld.pose.pose.position.y = 0;
  odomOld.pose.pose.orientation.z = 0;
  initialPoseRecieved = true;
}

// Calculate the distance the left wheel has traveled since the last cycle
void Calc_Left(const std_msgs::Int16& leftCount) {

  static int lastCountL = 0;
  if(leftCount.data != 0 && lastCountL != 0) {

    int leftTicks = (leftCount.data - lastCountL);
    if (leftTicks > 10000) { //underflow
      leftTicks = -65536 + leftTicks; //((-32768) - lastCountL) + (leftCount.data - 32767) - 1
    }
    else if (leftTicks < -10000) { //overflow
      leftTicks = 65536 + leftTicks; //(32767 - lastCountL) + (leftCount.data - (-32768)) + 1
    }
    else{}

    distanceLeft += leftTicks/TICKS_PER_METER;
  }
  lastCountL = leftCount.data;
}

// Calculate the distance the right wheel has traveled since the last cycle
void Calc_Right(const std_msgs::Int16& rightCount) {
  
  static int lastCountR = 0;
  if(rightCount.data != 0 && lastCountR != 0) {

    int rightTicks = rightCount.data - lastCountR;
    if (rightTicks > 10000) { //underflow
      rightTicks = -65536 + rightTicks; //((-32768) - lastCountL) + (leftCount.data - 32767) - 1
    }
    else if (rightTicks < -10000) { //overflow
      rightTicks = 65536 + rightTicks; //(32767 - lastCountL) + (leftCount.data - (-32768)) + 1
    }
    else{}

    distanceRight += rightTicks/TICKS_PER_METER;
  }
  lastCountR = rightCount.data;
}
void publish_quat() {

  nav_msgs::Odometry quatOdom;
  tf2::Quaternion odom_q;

  odom_q.setRPY(0, 0, odomNew.pose.pose.orientation.z);

  quatOdom.header.stamp = odomNew.header.stamp;
  quatOdom.header.frame_id = "odom";
  quatOdom.child_frame_id = "base_footprint";
  quatOdom.pose.pose.position.x = odomNew.pose.pose.position.x;
  quatOdom.pose.pose.position.y = odomNew.pose.pose.position.y;
  quatOdom.pose.pose.position.z = odomNew.pose.pose.position.z;
  quatOdom.pose.pose.orientation.x = odom_q.x();
  quatOdom.pose.pose.orientation.y = odom_q.y();
  quatOdom.pose.pose.orientation.z = odom_q.z();
  quatOdom.pose.pose.orientation.w = odom_q.w();
  quatOdom.twist.twist.linear.x = odomNew.twist.twist.linear.x;
  quatOdom.twist.twist.linear.y = odomNew.twist.twist.linear.y;
  quatOdom.twist.twist.linear.z = odomNew.twist.twist.linear.z;
  quatOdom.twist.twist.angular.x = odomNew.twist.twist.angular.x;
  quatOdom.twist.twist.angular.y = odomNew.twist.twist.angular.y;
  quatOdom.twist.twist.angular.z = odomNew.twist.twist.angular.z;

  for(int i = 0; i<36; i++) {
    if(i == 0 || i == 7 || i == 14) {
      quatOdom.pose.covariance[i] = .001;
     }
     else if (i == 21 || i == 28 || i== 35) {
       quatOdom.pose.covariance[i] += 0.001;
     }
     else {
       quatOdom.pose.covariance[i] = 0;
     }
  }


  // Covariance matrix
  // [0.01  0.0  0.0  0.0  0.0  0.0,
  //  0.0  0.01  0.0  0.0  0.0  0.0,
  //  0.0   0.0 0.01  0.0  0.0  0.0,
  //  0.0   0.0  0.0  0.1  0.0  0.0,
  //  0.0   0.0  0.0  0.0  0.1  0.0,
  //  0.0   0.0  0.0  0.0  0.0  0.1]



  // // tf2
  // static tf2_ros::StaticTransformBroadcaster base_link_br;
  // static tf2_ros::StaticTransformBroadcaster camera_link_br;
  // geometry_msgs::TransformStamped odom_transformStamped;
  // geometry_msgs::TransformStamped camera_link_transformStamped;

  // odom_transformStamped.header.stamp = odomNew.header.stamp;
  // odom_transformStamped.header.frame_id = "odom";
  // odom_transformStamped.child_frame_id = "base_link";
  // odom_transformStamped.transform.translation.x = quatOdom.pose.pose.position.x;
  // odom_transformStamped.transform.translation.y = quatOdom.pose.pose.position.y;
  // odom_transformStamped.transform.translation.z = quatOdom.pose.pose.position.z;
  // odom_transformStamped.transform.rotation.x = quatOdom.pose.pose.orientation.x;
  // odom_transformStamped.transform.rotation.y = quatOdom.pose.pose.orientation.y;
  // odom_transformStamped.transform.rotation.z = quatOdom.pose.pose.orientation.z;
  // odom_transformStamped.transform.rotation.w = quatOdom.pose.pose.orientation.w;
  // base_link_br.sendTransform(odom_transformStamped);

  // tf2::Quaternion camera_link_q;
  // camera_link_q.setRPY(0, 0, 0);

  // camera_link_transformStamped.header.stamp = odomNew.header.stamp;
  // camera_link_transformStamped.header.frame_id = "base_link";
  // camera_link_transformStamped.child_frame_id = "camera_link";
  // camera_link_transformStamped.transform.translation.x = 0.06;
  // camera_link_transformStamped.transform.translation.y = 0;
  // camera_link_transformStamped.transform.translation.z = 0.04;
  // camera_link_transformStamped.transform.rotation.x = camera_link_q.x();
  // camera_link_transformStamped.transform.rotation.y = camera_link_q.y();
  // camera_link_transformStamped.transform.rotation.z = camera_link_q.z();
  // camera_link_transformStamped.transform.rotation.w = camera_link_q.w();

  // camera_link_br.sendTransform(camera_link_transformStamped); 
  // odom_data_pub_quat.publish(quatOdom);
}
// Update odometry information
void update_odom() {
  // Calculate the average distance
  double cycleDistance = (distanceRight + distanceLeft) / 2;
  
  // Calculate the number of radians the robot has turned since the last cycle
  double cycleAngle = asin((distanceRight-distanceLeft)/WHEEL_BASE);

  // Initialize distance of a total loop
  distanceLeft = 0;
  distanceRight = 0;

  // Average angle during the last cycle
  double avgAngle = cycleAngle/2 + odomOld.pose.pose.orientation.z;

  if (avgAngle > PI) {
    avgAngle -= 2*PI;
  }
  else if (avgAngle < -PI) {
    avgAngle += 2*PI;
  }
  else{}

  // Calculate the new pose (x, y, and theta)
  odomNew.pose.pose.position.x = odomOld.pose.pose.position.x + cos(avgAngle)*cycleDistance;
  odomNew.pose.pose.position.y = odomOld.pose.pose.position.y + sin(avgAngle)*cycleDistance;
  odomNew.pose.pose.orientation.z = cycleAngle + odomOld.pose.pose.orientation.z;

  // Prevent lockup from a single bad cycle
  if (isnan(odomNew.pose.pose.position.x) || isnan(odomNew.pose.pose.position.y)
     || isnan(odomNew.pose.pose.position.z)) {
    odomNew.pose.pose.position.x = odomOld.pose.pose.position.x;
    odomNew.pose.pose.position.y = odomOld.pose.pose.position.y;
    odomNew.pose.pose.orientation.z = odomOld.pose.pose.orientation.z;
  }

  // Make sure theta stays in the correct range
  if (odomNew.pose.pose.orientation.z > PI) {
    odomNew.pose.pose.orientation.z -= 2 * PI;
  }
  else if (odomNew.pose.pose.orientation.z < -PI) {
    odomNew.pose.pose.orientation.z += 2 * PI;
  }
  else{}

  // Compute the velocity
  odomNew.header.stamp = ros::Time::now();
  odomNew.twist.twist.linear.x = cycleDistance/(odomNew.header.stamp.toSec() - odomOld.header.stamp.toSec());
  odomNew.twist.twist.angular.z = cycleAngle/(odomNew.header.stamp.toSec() - odomOld.header.stamp.toSec());

  // Save the pose data for the next cycle
  odomOld.pose.pose.position.x = odomNew.pose.pose.position.x;
  odomOld.pose.pose.position.y = odomNew.pose.pose.position.y;
  odomOld.pose.pose.orientation.z = odomNew.pose.pose.orientation.z;
  odomOld.header.stamp = odomNew.header.stamp;
  // ROS_INFO(odomNew.twist.twist.angular.z);
  // Publish the odometry message
  odom_data_euler_pub.publish(odomNew);
}

// Publish a nav_msgs::Odometry message in quaternion format


int main(int argc, char **argv) {

  // Set the data fields of the odometry message
  odomNew.pose.pose.position.z = 0;
  odomNew.pose.pose.orientation.x = 0;
  odomNew.pose.pose.orientation.y = 0;
  odomNew.twist.twist.linear.x = 0;
  odomNew.twist.twist.linear.y = 0;
  odomNew.twist.twist.linear.z = 0;
  odomNew.twist.twist.angular.x = 0;
  odomNew.twist.twist.angular.y = 0;
  odomNew.twist.twist.angular.z = 0;
  odomOld.pose.pose.position.x = initialX;
  odomOld.pose.pose.position.y = initialY;
  odomOld.pose.pose.orientation.z = initialTheta;

  // Launch ROS and create a node
  ros::init(argc, argv, "ekf_odom_pub");
  ros::NodeHandle node;

  // Subscribe to ROS topics
  ros::Subscriber subForRightCounts = node.subscribe("right_ticks", 100, Calc_Right, ros::TransportHints().tcpNoDelay());
  ros::Subscriber subForLeftCounts = node.subscribe("left_ticks", 100, Calc_Left, ros::TransportHints().tcpNoDelay());
  ros::Subscriber subInitialPose = node.subscribe("initial_2d", 100, set_initial_2d);

  // Publisher of simple odom message where orientation.z is an euler angle
  // odom_data_euler_pub = node.advertise<nav_msgs::Odometry>("odom_data_euler", 10);

  // Publisher of full odom message where orientation is quaternion
  odom_data_pub_quat = node.advertise<nav_msgs::Odometry>("odom_data_quat", 100);

  ros::Rate loop_rate(10);

  while(ros::ok()) {
    ros::spinOnce();
    if(initialPoseRecieved) {
      update_odom();
      publish_quat();
    }
    // ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
