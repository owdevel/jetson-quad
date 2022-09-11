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
#include <Eigen/Geometry>
#include <tf/transform_broadcaster.h>

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM3::System* mpSLAM;
    ros::Publisher pub;
    ros::Publisher depth;
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

    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::RGBD,bUseViewer);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;

    igb.pub = nh.advertise<geometry_msgs::PoseStamped>("/orbslam_pose", 100, true);
    igb.depth = nh.advertise<sensor_msgs::Image>("/orbslam_depth", 100, true);

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/color/image_raw", 100);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth/image_rect_raw", 100);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
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

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    Sophus::SE3f pose = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
    ROS_INFO("Pose: X: %f, Y:%f, Z:%f", pose.translation()(0), pose.translation()(1), pose.translation()(2));
    Eigen::Vector3f translation = pose.translation();
    Eigen::Quaternionf quaternion = pose.unit_quaternion();


    geometry_msgs::PoseStamped poseStamp;
    poseStamp.pose.position.x = translation.x();
    poseStamp.pose.position.y = translation.z();
    poseStamp.pose.position.z = translation.y();
    poseStamp.pose.orientation.w = quaternion.w();
    poseStamp.pose.orientation.x = quaternion.x();
    poseStamp.pose.orientation.y = quaternion.z();
    poseStamp.pose.orientation.z = quaternion.y();

    poseStamp.header.stamp = msgD->header.stamp;
    poseStamp.header.frame_id = msgD->header.frame_id;

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(poseStamp.pose.position.x, poseStamp.pose.position.y, poseStamp.pose.position.z));
    tf::Quaternion q;
    q.setW(poseStamp.pose.orientation.w);
    q.setX(poseStamp.pose.orientation.x);
    q.setY(poseStamp.pose.orientation.y);
    q.setZ(poseStamp.pose.orientation.z);
    transform.setRotation(q);

    tf::StampedTransform stf;
    stf.setData(transform);
    stf.stamp_ = msgD->header.stamp;
    stf.frame_id_ = "map";
    //stf.child_frame_id_ = "camera_link";
    stf.child_frame_id_ = msgD->header.frame_id;

    br.sendTransform(stf);

    pub.publish(poseStamp);

    depth.publish(msgD);




}


