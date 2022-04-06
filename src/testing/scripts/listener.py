#!/usr/bin/env python

import rospy
import mavros_msgs.msg
import mavros_msgs.srv
import sensor_msgs.msg


def callback(data):
    print(data)


def main():
    #rospy.Subscriber('mavros/state', mavros_msgs.msg.State, callback)

    # Set Stream Rate of data calls for mavros
    set_stream_rate = rospy.ServiceProxy('/mavros/set_stream_rate',
                                         mavros_msgs.srv.StreamRate)

    # Stream 0 (maybe linked to modes?), 10Hz, Enable
    set_stream_rate(0, 10, 1)

    rospy.Subscriber('mavros/imu/data', sensor_msgs.msg.Imu, callback)


if __name__ == '__main__':
    try:
        rospy.init_node('listener', anonymous=True)

        main()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass