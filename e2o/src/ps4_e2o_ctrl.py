#!/usr/bin/env python
import rospy
from math import pi
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from e2o.msg import e2o_ctrl, e2o_status
car_control = e2o_ctrl()

car_control.Steer = 0
car_control.Accel = 0
car_control.Brake = 0
car_control.RNDB = "N"
car_control.Horn = 0

def ps4_callback(data, pub):
  global car_control
  car_control.Steer = int(round(data.axes[0]*180/pi, 4))

  if car_control.RNDB == "N":
    car_control.Accel = 0
  else:
    car_control.Accel = 100 - int(round((data.axes[2]+1)/2, 4)*100)

  car_control.Brake = int(round((data.axes[5]+1)/2, 4)*100)
  
  joy_btn_x = data.buttons[0]
  joy_btn_circ = data.buttons[1]
  joy_btn_tri = data.buttons[2]
  joy_btn_sqr = data.buttons[3]
  joy_gear = [joy_btn_sqr, joy_btn_tri, joy_btn_x, joy_btn_circ, ]
      
  
  if car_control.Steer >= 40:
    car_control.Steer = 40
  elif car_control.Steer <= -40:
    car_control.Steer = -40
  
  if joy_gear[0]==1:
    car_control.RNDB = "N"
  if joy_gear[1]==1:
    car_control.RNDB = "D"
  if joy_gear[2]==1:
    car_control.RNDB = "R"
  if joy_gear[3]==1:
    car_control.RNDB = "B"

  e2octrlpub.publish(car_control)

if __name__ == '__main__':
    rospy.init_node('ps4_e2o_ctrl', anonymous=True)
    e2octrlpub = rospy.Publisher('e2octrl', e2o_ctrl, queue_size=10)
    rospy.Subscriber("joy", Joy, ps4_callback, e2octrlpub)
    rospy.spin()