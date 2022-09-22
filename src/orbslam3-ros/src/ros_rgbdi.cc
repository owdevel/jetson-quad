/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with ORB-SLAM3.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <mutex>
#include <queue>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core/core.hpp>

#include "System.h"

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Geometry>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

using namespace std;

class ImuGrabber
{
public:
    ImuGrabber(){};
    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

    queue<sensor_msgs::ImuConstPtr> imuBuf;
    std::mutex mBufMutex;
};

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System *pSLAM, ImuGrabber *pImuGb) : mpSLAM(pSLAM), mpImuGb(pImuGb), listener(ros::Duration(10)) {}

    void GrabRGBD(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD);

    ORB_SLAM3::System *mpSLAM;
    ImuGrabber *mpImuGb;
    tf::TransformListener listener;

    ros::Publisher pub;
    ros::Publisher depth;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if (argc != 4)
    {
        cerr << endl
             << "Usage: rosrun ORB_SLAM3 RGBD path_to_vocabulary path_to_settings gui" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.

    const string tr = "true";
    const bool bUseViewer = tr.compare(argv[3]) == 0;

    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::RGBD, bUseViewer);

    ImuGrabber imugb;
    ImageGrabber igb(&SLAM, &imugb);

    ros::NodeHandle nh;

    ros::Subscriber sub_imu = nh.subscribe("/camera/imu", 1000, &ImuGrabber::GrabImu, &imugb);

    igb.pub = nh.advertise<geometry_msgs::PoseStamped>("/orbslam_pose", 100, true);
    igb.depth = nh.advertise<sensor_msgs::Image>("/orbslam_depth", 100, true);

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/color/image_raw", 100);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth/image_rect_raw", 100);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub, depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD, &igb, _1, _2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    double tIm = msgRGB->header.stamp.toSec();

    vector<ORB_SLAM3::IMU::Point> vImuMeas;
    mpImuGb->mBufMutex.lock();
    if (!mpImuGb->imuBuf.empty())
    {
        // Load imu measurements from buffer
        vImuMeas.clear();
        while (!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec() <= tIm)
        {
            double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
            cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);
            cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);
            vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
            mpImuGb->imuBuf.pop();
        }
    }
    mpImuGb->mBufMutex.unlock();

    Sophus::SE3f pose = mpSLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, cv_ptrRGB->header.stamp.toSec(), vImuMeas);

    // ROS_INFO("Pose: X: %f, Y:%f, Z:%f", pose.translation()(0), pose.translation()(1), pose.translation()(2));
    Eigen::Vector3f translation = pose.translation();
    Eigen::Quaternionf quaternion = pose.unit_quaternion();

    try
    {
        tf::StampedTransform transform;
        listener.lookupTransform("base_link", "camera_link",
                                  ros::Time(0), transform);

        tf::Transform cameratf;
        cameratf.setOrigin(tf::Vector3(translation.x(), translation.y(), translation.z()));
        cameratf.setRotation(tf::Quaternion(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w()));


        tf::Transform maptf = transform * cameratf * transform.inverse();
        
        tf::StampedTransform stf;
        stf.stamp_ = msgD->header.stamp;
        stf.setOrigin(maptf.getOrigin());
        stf.setRotation(maptf.getRotation());
        stf.frame_id_ = "map";
        stf.child_frame_id_ = "base_link";

        ROS_INFO("MapPose: X: %f, Y:%f, Z:%f", stf.getOrigin()[0], stf.getOrigin()[1], stf.getOrigin()[2]);


        static tf::TransformBroadcaster br;
        br.sendTransform(stf);


        depth.publish(msgD);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
    }
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
    mBufMutex.lock();
    imuBuf.push(imu_msg);
    mBufMutex.unlock();
    return;
}
