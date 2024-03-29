#!/usr/bin/env python

import rospy
import mavros_msgs.msg
import mavros_msgs.srv
import sensor_msgs.msg


def callback(data):
    print(data)


def main():
    print('Subscribing to State')
    rospy.Subscriber('mavros/state', mavros_msgs.msg.State, callback)

    # print('Setting stream rate')
    # # Set Stream Rate of data calls for mavros
    # set_stream_rate = rospy.ServiceProxy('/mavros/set_stream_rate',
    #                                      mavros_msgs.srv.StreamRate)

    # # Stream 0 (maybe linked to modes?), 10Hz, Enable
    # set_stream_rate(0, 200, 1)
    set_message_interval = rospy.ServiceProxy('/mavros/set_message_interval', mavros_msgs.srv.MessageInterval)

    print('Subscribing to IMU')
    rospy.Subscriber('mavros/imu/data', sensor_msgs.msg.Imu, callback)


if __name__ == '__main__':
    try:
        rospy.init_node('listener', anonymous=True)

        main()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
