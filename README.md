# dlive_project
# creating a gazebo model for the mahindra e2o car

# Launch a keyboard teleop for ackermann drive control
./run_gazebo.sh

#or

roslaunch ackermann_vehicle_gazebo ackermann_vehicle_teleop.launch

#Use W,A,S,D to move around 

imu readings published on the topic /imu
To get the image output: rosrun image_view image_view image:=/mybot/camera1/image_raw


