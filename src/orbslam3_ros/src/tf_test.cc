#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

// Playground file for finding out how to transform to world frame in ROS.

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_test");

  ros::NodeHandle node;

  tf::TransformListener listener;

  ros::Rate rate(10.0);

  while (node.ok()){
        geometry_msgs::PoseStamped cameraPose;
        // cameraPose.pose.position.x = 0;
        cameraPose.pose.position.x = 1;
        cameraPose.pose.position.y = 0;
        cameraPose.pose.position.z = 0;

        cameraPose.pose.orientation.x = 0;
        // cameraPose.pose.orientation.y = -0.6816388;
        cameraPose.pose.orientation.y = 0;
        cameraPose.pose.orientation.z = 0;
        // cameraPose.pose.orientation.w = 0.7316889;
        cameraPose.pose.orientation.w = 1;
        cameraPose.header.frame_id = "camera_link";
        // pub.publish(cameraPose);

    try {
        tf::StampedTransform transform;
        listener.lookupTransform("base_link", "camera_link",
                                  ros::Time(0), transform);

        tf::Stamped<tf::Transform> cameratf;
        tf::poseStampedMsgToTF(cameraPose, cameratf);

        tf::Transform maptf = transform * cameratf * transform.inverse();
        
        tf::StampedTransform stf;
        stf.stamp_ = cameraPose.header.stamp;
        stf.setOrigin(maptf.getOrigin());
        stf.setRotation(maptf.getRotation());
        stf.frame_id_ = "map";
        stf.child_frame_id_ = "base_link";

        ROS_INFO("MapPose: X: %f, Y:%f, Z:%f", stf.getOrigin()[0], stf.getOrigin()[1], stf.getOrigin()[2]);


        static tf::TransformBroadcaster br;
        br.sendTransform(stf);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    rate.sleep();
  }
  return 0;
};
