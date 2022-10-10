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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include "System.h"

#include <geometry_msgs/PoseStamped.h>
#include "orbslam3_ros/SLAMTime.h"
#include <Eigen/Geometry>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msgRGB);

    ORB_SLAM3::System* mpSLAM;
    tf::TransformListener listener;

    ros::Subscriber sub;

    ros::Publisher slamTimePub;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 RGBD path_to_vocabulary path_to_settings gui" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.

    const string tr = "true";
    const bool bUseViewer = tr.compare(argv[3]) == 0;

    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,bUseViewer);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;

    igb.slamTimePub = nh.advertise<orbslam3_ros::SLAMTime>("/orbslam_time", 100, true);

    igb.sub = nh.subscribe("/camera/color/image_raw", 100, &ImageGrabber::GrabImage, &igb);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    // SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msgRGB)
{

    ros::Time start_time = ros::Time::now();

    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Sophus::SE3f pose = mpSLAM->TrackMonocular(cv_ptrRGB->image,cv_ptrRGB->header.stamp.toSec());

    Eigen::Vector3f translation = pose.translation();
    Eigen::Quaternionf quaternion = pose.unit_quaternion();

    try
    {
        tf::StampedTransform transform;
        listener.lookupTransform("base_link", "camera_depth_optical_frame",
                                  ros::Time(0), transform);

        // Need to change the axis as ORBSLAM uses left-handed coordinate frames and ROS uses right-handed
        // to match camera_depth_optical_frame, invert all the axis
        tf::Transform cameratf;
        cameratf.setOrigin(tf::Vector3(-translation.x(), -translation.y(), -translation.z()));
        cameratf.setRotation(tf::Quaternion(-quaternion.x(), -quaternion.y(), -quaternion.z(), quaternion.w()));

        ROS_INFO("OrbPose: X: %f, Y:%f, Z:%f", translation.x(), translation.y(), translation.z());


        tf::Transform maptf = transform * cameratf * transform.inverse();
        
        tf::StampedTransform stf;
        stf.stamp_ = msgRGB->header.stamp;
        stf.setOrigin(maptf.getOrigin());
        stf.setRotation(maptf.getRotation());
        stf.frame_id_ = "map";
        stf.child_frame_id_ = "base_link";

        ROS_INFO("MapPose: X: %f, Y:%f, Z:%f", stf.getOrigin()[0], stf.getOrigin()[1], stf.getOrigin()[2]);


        static tf::TransformBroadcaster br;
        br.sendTransform(stf);


        //depth.publish(msgRGB);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
    }

    ros::Time stop_time = ros::Time::now();
    ros::Duration diff = stop_time - start_time;

    orbslam3_ros::SLAMTime slamTime;
    slamTime.header = msgRGB->header;
    slamTime.start_time = start_time;
    slamTime.diff = diff;

    slamTimePub.publish(slamTime);
}


