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

prev_accel = 0


def joy_callback(data, pub):
  global car_control
  global prev_accel

  car_control.Steer = int(round(data.axes[0]*180/pi, 4))
  
  if car_control.RNDB == "N":
    car_control.Accel = 0
  else:
    car_control.Accel = int(round((data.axes[2]+1)/2, 4)*100)
    if (car_control.Accel - prev_accel) > 5:
      car_control.Accel = prev_accel
    else:
      prev_accel = car_control.Accel
  #print("Acceleration : " + str(car_control.Accel))
  
  car_control.Brake = int(round((data.axes[3]+1)/2, 4)*100)
  
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
  joy_btn_enter = data.buttons[23]
  
  if car_control.Steer >= 40:
    car_control.Steer = 40
  elif car_control.Steer <= -40:
    car_control.Steer = -40
  
  if joy_gear[0]==0 and joy_gear[1]==0 and joy_gear[2]==0:
    car_control.RNDB = "N"
  if joy_gear[0]==1:
    car_control.RNDB = "D"
  if joy_gear[1]==1:
    car_control.RNDB = "R"
  if joy_gear[2]==1:
    car_control.RNDB = "B"

  car_control.Horn = joy_btn_enter

  e2octrlpub.publish(car_control)

if __name__ == '__main__':
    rospy.init_node('logitech_e2o_ctrl', anonymous=False)
    e2octrlpub = rospy.Publisher('e2octrl', e2o_ctrl, queue_size=10)
    rospy.Subscriber("joy", Joy, joy_callback, e2octrlpub)
    rospy.spin()