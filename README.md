# dlive_project
# creating a gazebo model for the mahindra e2o car

# Launch a keyboard teleop for ackermann drive control
roslaunch ackermann_vehicle_gazebo ackermann_vehiclekeyboard_leop.launch 

Use W,A,S,D to move around 
# Sensor Readings
imu readings published on the topic /imu, to view the imu readings
rostopic echo /imu

There's two camera attached to the model:-
  1)Monocamera:

  To get the image output: rosrun image_view image_view image:=/mybot/camera1/image_raw

  2)Stereocamera:

  rosrun image_view image_view image:=/multisense_sl/camera/right/image_raw
  rosrun image_view image_view image:=/multisense_sl/camera/left/image_raw

# For PID steering controller-

roslaunch test_scripts pid_auto_testing.launch 

It may happen that this .launch file may not work. In that case, you will need to start the following processes in different command windows (in the same order):

1)roslaunch ackermann_vehicle_gazebo ackermann_vehicle_teleop.launch

2)rosrun test_scripts find_angle.py 

3)rosrun pid controller 

4)rosrun rqt_reconfigure rqt_reconfigure 

5)rosrun rqt_robot_monitor rqt_robot_monitor

6)rosrun test_scripts pid_test.py

7)rqt_plot

You can change the values of Kp, Ki and Kd by going to the controller tab in the rqt_reconfigure window

To Autotune the pid controller using Zeigler Nichols method, run the following command in a new terminal

rosrun pid autotune

Just in case you are having some problem while dowloading from the github repository, use the following link to download the catkin workspace

https://drive.google.com/drive/folders/1iEfKzO4zGhvi1kfcKMo-ledSxtuS4qck?usp=sharing

