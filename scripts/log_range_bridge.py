#!/usr/bin/env python
# Converts the Crayflie driver generic log of the ranging to a RangeArray msg
# The log data packet is assumed to contain:
# range0..5, status
import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import Float32MultiArray

from crazyflie_driver.msg import GenericLogData

def callback(data):
    ranges = Range()

    ranges.header.frame_id = rospy.get_namespace() + "base_link"
    #ranges.header.frame_id = "/Crazyflie1/base_link"
    ranges.header.stamp = rospy.Time.now()
    ranges.radiation_type = Range.INFRARED;
    ranges.field_of_view = 3.14159/9
    ranges.min_range = 0
    ranges.max_range = 1.2

    ranges.range = (data.data[0]/1000)

    range_pub.publish(ranges)

def vel_cb(data):
    z = Float32()
    z.data = data.data[2]
    vel_z_pub.publish(z)

def vel_sp_cb(data):
    z = Float32()
    z.data = data.data[2]
    vel_z_sp_pub.publish(z)

if __name__ == "__main__":
    rospy.init_node('log_range_bridge')

    range_pub = rospy.Publisher(rospy.get_namespace()+"range_front", Range, queue_size=10)

    rospy.Subscriber(rospy.get_namespace()+"log_range", Float32MultiArray, callback)

    rospy.spin()
