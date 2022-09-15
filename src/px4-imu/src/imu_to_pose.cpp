
#include "ros/ros.h"
#include "std_msgs/String.h"

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>

#include "Fusion.h"

#include <sstream>

class ImuToPose
{
public:
    ImuToPose(ros::NodeHandle n);

    void processIMU(const nav_msgs::OdometryConstPtr& msgOdom);

    ros::Publisher pub;
    ros::Subscriber sub;
    geometry_msgs::PoseStamped pose;

    FusionAhrs ahrs;
};

ImuToPose::ImuToPose(ros::NodeHandle n)
{
    ROS_INFO("Subscribing to pose");
    sub = n.subscribe<nav_msgs::Odometry>("/mavros/global_position/local", 100, &ImuToPose::processIMU, this);
    pub = n.advertise<geometry_msgs::PoseStamped>("/px4_pose", 100, true);

    pose.pose.position.x = 0;
    pose.pose.position.y = 0;

    pose.header.frame_id = "base_link";
    pose.header.stamp = ros::Time::now();
}

void ImuToPose::processIMU(const nav_msgs::OdometryConstPtr& msgOdom)
{

    ros::Time last_time = pose.header.stamp;
    ros::Time current_time = msgOdom->header.stamp;
    ros::Duration ros_dt = current_time - last_time;
    float dt = ros_dt.nsec / 1000000000.0;

    pose.header.stamp = msgOdom->header.stamp;

    pose.pose.orientation = msgOdom->pose.pose.orientation;
    pose.pose.position.z = msgOdom->pose.pose.position.z;

    pose.pose.position.x += msgOdom->twist.twist.linear.x;
    pose.pose.position.y += msgOdom->twist.twist.linear.y;

    ROS_INFO("Publishing Pose: x %f y %f z %f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    pub.publish(pose);

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
    tf::Quaternion q;
    q.setW(pose.pose.orientation.w);
    q.setX(pose.pose.orientation.x);
    q.setY(pose.pose.orientation.y);
    q.setZ(pose.pose.orientation.z);
    transform.setRotation(q);

    tf::StampedTransform stf;
    stf.setData(transform);
    stf.stamp_ = pose.header.stamp;
    stf.frame_id_ = "map";
    stf.child_frame_id_ = pose.header.frame_id;

    br.sendTransform(stf);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_to_pose");

    ros::NodeHandle n;

    ImuToPose imu2pose(n);

    ros::spin();
}