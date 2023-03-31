# leader_follower_Robot

The project is done using the Melodic ROS

#General discription
The used model of the robots: Turtulebot3 Waffle_pi
The project contains one leader robot moving in an arbitrary trajectory depending on keyboard, and the other robot is following it according to the data obtained from camera, and laser Scanner.
From the camera we extract the center of the robot and calculate the required twist need for the follower.
And the laser scanner measure the distance between the robots and keeps a minimum distance between them.




## Getting started


```
cd existing_repo
git remote add origin https://gitlab.com/yazanMaalla/leader_follower_robot.git
git branch -M main
git push -uf origin main



```
then after building the workspace:


```
export TURTLEBOT3_MODEL=Waffle_pi
sudo apt-get install ros_melodic_turtlebot3_description
sudo apt-get install ros_melodic_turtlebot3_teleop
roslaunch leader_follower leader.launch

```

and in a new terminal:

```
export TURTLEBOT3_MODEL=Waffle_pi
rosrun leader_follower camera_follow.py

```

maintainers.
