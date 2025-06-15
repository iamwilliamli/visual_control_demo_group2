#!/usr/bin/env python2

import rospy
from std_msgs.msg import Int32MultiArray
from encoderDriver import WheelEncoderDriver
from time import sleep

def run_encoder():
    pub = rospy.Publisher('encoder_ticks', Int32MultiArray, queue_size=10)
    rospy.init_node('encoder_node', anonymous=True)
    rate = rospy.Rate(20)  # 50ms = 20Hz

    encoder_r = WheelEncoderDriver(18)
    encoder_l = WheelEncoderDriver(19)

    while not rospy.is_shutdown():
        ticks_msg = Int32MultiArray()
        ticks_msg.data = [encoder_r._ticks, encoder_l._ticks]
        pub.publish(ticks_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        run_encoder()
    except rospy.ROSInterruptException:
        pass