## installing docker - [ref](https://phoenixnap.com/kb/how-to-install-docker-on-ubuntu-18-04)
 
1. sudo apt install docker.io
1. sudo systemctl start docker
1. sudo systemctl enable docker

## ros2 docker - [ref](https://discourse.ros.org/t/announcing-ros-foxy-docker-images-availability/14511)

1. (only for raspberry) sudo docker pull arm64v8/ros:foxy-ros-base-focal
2. sudo docker pull amd64/ros:foxy-ros-base-focal
3. sudo docker create -it --name maila -h maila amd64/ros:foxy-ros-base-focal
4. sudo docker start -ia maila

to not use sudo:
1. sudo groupadd docker
1. sudo usermod -aG docker $USER

 

## installing VisualStudio Code

1. form ubuntu software
2. extensions:
   1. ROS (0.6.4)
   2. Markdown All in One

## ros2 turtlesim example

1. source /opt/ros/foxy/setup.bash
2. printenv | grep -i ROS
3. export ROS_DOMAIN_ID=25
4. apt update
5. apt install ros-foxy-turtlesim
6. ros2 pkg executables turtlesim
7. ros2 run turtlesim turtlesim_node
8. ros2 run turtlesim turtle_teleop_key