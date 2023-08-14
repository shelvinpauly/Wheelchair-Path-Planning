#new terminal
cd catkin_ws
catkin_make
source devel/setup.bash
roslaunch Final_gazebo turtlebot3_empty_world.launch

# new terminal(ctrl+shift+t)
cd catkin_ws/Final_gazebo
python3 publish_proj5.py


'''
Challenges faced:
Dynamic obstacles could not be implemented in gazebo. It is only shown in opencv.
'''