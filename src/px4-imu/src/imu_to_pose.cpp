
#include "ros/ros.h"
#include "std_msgs/String.h"

#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"

#include <sstream>

class ImuToPose
{
public:
    ImuToPose(ros::NodeHandle n);

    void processIMU(const sensor_msgs::ImuConstPtr& msgIMU);

    ros::Publisher pub;
    ros::Subscriber sub;
    geometry_msgs::PoseStamped pose;
};

ImuToPose::ImuToPose(ros::NodeHandle n)
{
    sub = n.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 100, &ImuToPose::processIMU, this);

    pose.pose.orientation.w = 1;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;

    pose.header.frame_id = "base_link";
    pose.header.stamp = ros::Time::now();

}

void ImuToPose::processIMU(const sensor_msgs::ImuConstPtr& msgIMU)
{
    ROS_INFO("IMU: %f", msgIMU->linear_acceleration.x);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_to_pose");

    ros::NodeHandle n;

    ImuToPose imu2pose(n);

    ros::spin();
}