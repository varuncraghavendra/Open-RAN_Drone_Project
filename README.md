# nwconfig_drone

Inside px4_agent_ws directory, 

Step 1 : Setup PX4 Docker

`sudo docker build -t px4_sitl_img ./px4_docker_images/px4_sitl_img`

Step 2 : Setup ROS2 Docker 

`sudo docker build -t ros2_humble_img ./px4_docker_images/ros2_humble_img`

Step 3 : Run the PX4 Container from the px4_docker_images

`sudo docker run -it --rm px4_sitl_img /bin/bash`

Step 4 : Run Docker from px4_agent_ws/px4_docker_images

`sudo docker run -it      --privileged     --env="DISPLAY"     --workdir="/home/ros2_ws/"     --volume="/dev:/dev"     --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"     --volume="/var/run/dbus:/var/run/dbus"     --volume="./src:/home/ros2_ws/src"     --network host     --name $CONTAINER_NAME     $IMAGE_NAME     "$@"`



Step 5 : 



NOTE : Remove existing docker

`sudo docker rm tii_ros2_agent`

To stop docker : 

`docker stop px4-dev-test`



## SSH Key Setup 

Step 1 : Generate a New SSH Key (if none exists)

`ssh-keygen -t ed25519 -C "your_email@example.com"`

Step 2 : Then start the SSH agent and add your key:

eval "$(ssh-agent -s)"

`ssh-add ~/.ssh/id_ed25519`

Step 3 : Add SSH Key to Bitbucket

`cat ~/.ssh/id_ed25519.pub # Copy your public key
`
Step 4 : Go to Bitbucket:

Click on your avatar → Personal settings

Go to SSH keys

Click Add key

Paste the public key and save

### Docker Management 

`sudo docker ps -a` # Lists down all current and past dockers

`sudo docker exec -it` # Restart the docker


 





