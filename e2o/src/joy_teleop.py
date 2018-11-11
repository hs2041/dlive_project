#!/usr/bin/env python
import rospy
from math import pi
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

def joy_callback(data):
  joy_steering_angle = int(round(data.axes[0]*180/pi, 4))
  joy_pedal_accel = int(round((data.axes[2]+1)/2, 4)*100)
  joy_pedal_brake = int(round((data.axes[3]+1)/2, 4)*100)
  joy_gear_state = "Neutral"
  joy_gear_1 = data.buttons[12]
  joy_gear_2 = data.buttons[13]
  joy_gear_3 = data.buttons[14]
  joy_gear_4 = data.buttons[15]
  joy_gear_5 = data.buttons[16]
  joy_gear_r = data.buttons[17]
  joy_gear = [joy_gear_1, joy_gear_2, joy_gear_4]
  joy_btn_l2 = data.buttons[7]
  joy_btn_r2 = data.buttons[6]
  joy_btn_l3 = data.buttons[11]
  joy_btn_r3 = data.buttons[10]
  joy_btn_x = data.buttons[0]
  joy_btn_sqr = data.buttons[1]
  joy_btn_circ = data.buttons[2]
  joy_btn_tri = data.buttons[3]
  if joy_steering_angle >= 40:
    joy_steering_angle = 40
  elif joy_steering_angle <= -40:
    joy_steering_angle = -40
  if joy_gear[0]==0 and joy_gear[1]==0 and joy_gear[2]==0:
    joy_gear_state = "Neutral"
  if joy_gear[0]==1:
    joy_gear_state = "Forward"
  if joy_gear[1]==1:
    joy_gear_state = "Reverse"
  if joy_gear[2]==1:
    joy_gear_state = "Boost"
  print joy_gear
  print "Steering: ", joy_steering_angle
  print "Accel: ", joy_pedal_accel, "%"
  print "Brake: ", joy_pedal_brake,"%"
  print joy_gear_state
    
def joy_teleop():
    rospy.init_node('joy_teleop', anonymous=True)
    rospy.Subscriber("joy", Joy, joy_callback)
    rospy.spin()

if __name__ == '__main__':
    joy_teleop()
