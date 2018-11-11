#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

new_data = LaserScan()
num_readings = 180
start_angle = 90
laser_frequency = 10

def scan_callback(data, r2000_scan):
    global new_data
    current_time = rospy.Time.now()
    new_data.header.stamp = current_time
    new_data.header.frame_id = 'laser_frame'
    new_data.angle_min = -1.57
    new_data.angle_max = 1.57
    new_data.angle_increment = 3.14 / num_readings
    new_data.time_increment = (1.0 / laser_frequency) / (num_readings)
    new_data.range_min = 0.0
    new_data.range_max = 100.0

    new_data.ranges = []
    new_data.intensities = []
    for i in range(start_angle, start_angle + num_readings):
        new_data.ranges.append(data.ranges[i])
        new_data.intensities.append(data.intensities[i])
    
    r2000_scan.publish(new_data)
    #print ("range ahead: %0.1f" %range_ahead)

rospy.init_node('r2000_update')
r2000_scan = rospy.Publisher('r2000_scan', LaserScan, queue_size = 1)
scan_sub = rospy.Subscriber('/r2000_node/scan', LaserScan, scan_callback, r2000_scan)
rospy.spin()