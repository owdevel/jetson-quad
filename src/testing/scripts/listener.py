#!/usr/bin/env python

import rospy
import mavros_msgs.msg
import sensor_msgs.msg


def callback(data):
    print(data)


def main():
    #rospy.Subscriber('mavros/state', mavros_msgs.msg.State, callback)
    rospy.Subscriber('mavros/imu/data', sensor_msgs.msg.Imu, callback)


if __name__ == '__main__':
    try:
        rospy.init_node('listener', anonymous=True)

        main()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass