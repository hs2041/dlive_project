#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from e2o.msg import e2o_status
import socket
import time
UDP_IP_ADDRESS ="182.71.128.38" 
UDP_PORT_NO = 8888


import socket
import time

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    clientSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    Message = "%s %s"%(data,str(time.time()*1000))
    ret=clientSock.sendto(Message, (UDP_IP_ADDRESS, UDP_PORT_NO))
    print ret

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('ros_udpnode', anonymous=True)

    rospy.Subscriber("e2ostatus", e2o_status, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()


