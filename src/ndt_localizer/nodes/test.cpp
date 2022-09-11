#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <sstream>
#include <string>
#include <ros/ros.h>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>

#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "test");
    ros::NodeHandle n;
    ros::Publisher pub_ = n.advertise<nav_msgs::Odometry>("result", 10);
    ros::Rate loop_rate(10);
    int count = 0;
    while (ros::ok())
    {
        nav_msgs::Odometry odom_msg;
        odom_msg.pose.pose.position.x = 1;
        odom_msg.pose.pose.position.y = 0;
        odom_msg.pose.pose.position.z = 0;
        odom_msg.pose.pose.orientation.x = 0;
        odom_msg.pose.pose.orientation.y = 0;
        odom_msg.pose.pose.orientation.z = 0.707;
        odom_msg.pose.pose.orientation.w = 0.707;

        geometry_msgs::Quaternion orientation = odom_msg.pose.pose.orientation;
        geometry_msgs::Point position = odom_msg.pose.pose.position;

        Eigen::Quaterniond quat;
        quat.w() = orientation.w;
        quat.x() = orientation.x;
        quat.y() = orientation.y;
        quat.z() = orientation.z;

        Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
        isometry.linear() = quat.toRotationMatrix();
        isometry.translation() = Eigen::Vector3d(position.x, position.y, position.z);

        Eigen::Matrix4f m = isometry.matrix().cast<float>();
        std::cout << m << std::endl;
        std::cout << "****************************" << std::endl;

    }
    return 0;

}
