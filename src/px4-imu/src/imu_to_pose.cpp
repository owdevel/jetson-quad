
#include "ros/ros.h"
#include "std_msgs/String.h"

#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"

#include "Fusion.h"

#include <sstream>

class ImuToPose
{
public:
    ImuToPose(ros::NodeHandle n);

    void processIMU(const sensor_msgs::ImuConstPtr& msgIMU);

    ros::Publisher pub;
    ros::Subscriber sub;
    geometry_msgs::PoseStamped pose;

    FusionAhrs ahrs;
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

    FusionAhrsInitialise(&ahrs);

}

void ImuToPose::processIMU(const sensor_msgs::ImuConstPtr& msgIMU)
{
    const float G = 9.80665;

    FusionVector gyroscope = {0.0f, 0.0f, 0.0f}; // replace this with actual gyroscope data in degrees/s
    FusionVector accelerometer = {0.0f, 0.0f, 1.0f}; // replace this with actual accelerometer data in g

    gyroscope.array[0] = msgIMU->angular_velocity.x * 180 / M_PI;
    gyroscope.array[1] = msgIMU->angular_velocity.y * 180 / M_PI;
    gyroscope.array[2] = msgIMU->angular_velocity.z * 180 / M_PI;


    accelerometer.array[0] = msgIMU->linear_acceleration.x / 9.81;
    accelerometer.array[1] = msgIMU->linear_acceleration.y / 9.81;
    accelerometer.array[2] = msgIMU->linear_acceleration.z / 9.81;

    ros::Time last_time = pose.header.stamp;
    ros::Time current_time = msgIMU->header.stamp;

    ros::Duration diff = current_time - last_time;
    float dt = diff.nsec / 1000000000;


    FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, dt);

    const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
    const FusionVector earth = FusionAhrsGetEarthAcceleration(&ahrs);

    ROS_INFO("Roll %0.1f, Pitch %0.1f, Yaw %0.1f, X %0.1f, Y %0.1f, Z %0.1f",
           euler.angle.roll, euler.angle.pitch, euler.angle.yaw,
           earth.axis.x, earth.axis.y, earth.axis.z);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_to_pose");

    ros::NodeHandle n;

    ImuToPose imu2pose(n);

    ros::spin();
}