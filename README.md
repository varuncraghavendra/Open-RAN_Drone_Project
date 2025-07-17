## Date and time syncup

`sudo date -s "2025-07-02 14:30:00"`


## Run the Simulation 

T1 : `MicroXRCEAgent udp4 -p 8888`

T2 : `ros2 launch system_bringup launch_nodes.launch.py `

T3 : `ros2 run goal_manager goal_manager_server`

T4 : `ros2 launch drone_basic_control launch_all_nodes.launch.py`

#### In PX4 Terminal

`PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4`


## Re-Enter the dockers 

#### px4-dev-test

`docker start px4-dev-test`

`docker exec -it px4-dev-test bash`

#### px4_agent_ws

`docker start px4_agent_ws`

`docker exec -it px4_agent_ws bash`

#### Source the /home/ros2_ws inside docker

`source install/setup.bash`

#### Launch Micro XRCE Agent and ROS2 Nodes

`MicroXRCEAgent udp4 -p 8888`

#### Open a new terminal and run the following 

`./run_container.sh`

`source install/setup.bash`

`ros2 launch drone_basic_control launch_all_nodes.launch.py`


#### Open another new terminal and run the following 

`./run_container.sh`

`source install/setup.bash`

`ros2 launch system_bringup launch_nodes.launch.py `


## Step By Step Instructions for first time build

`sudo apt update`

`sudo apt install -y docker.io docker-compose git x11-xserver-utils`

`sudo usermod -aG docker $USER`

`newgrp docker`  # Apply docker group without reboot

`git clone --recurse-submodules git@bitbucket.org:autonomouscv/px4_agent_ws.git` # Clone the repository

`cd px4_agent_ws`

### Build Docker Images
#### From root of the workspace:

#### PX4 SITL Container

`cd px4_docker_images/px4_sitl_img`

`sudo docker build -t px4_sim .`

`cd ../ros2_humble_img`

`sudo docker build -t ros2-humble-ntt .`


#### Run Docker Containers

`cd px4_docker_images/px4_sitl_img`

`chmod +x start.sh`

`./start.sh`

#### Let PX4 SITL stay running.

### ROS 2 Container

`cd px4_docker_images/ros2_humble_img`

`chmod +x run_container.sh`

`./run_container.sh`

#### Inside the ROS 2 Container

`pip3 install --user -U empy==3.3.4 pyros-genmsg setuptools==70.3.0`

`pip install py_trees typing_extensions`

`apt-get update`

`apt-get install -y ros-humble-navigation2`

#### Install Micro XRCE DDS Agent

`cd /home`

`git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git`

`cd Micro-XRCE-DDS-Agent`

`mkdir build && cd build`

`cmake ..`

`make`

`sudo make install`

`sudo ldconfig /usr/local/lib/`

#### Add ROS 2 Setup to Bash

`nano ~/.bashrc`

`source /opt/ros/humble/setup.bash`

`source /home/ros2_ws/install/setup.bash`

#### Build ROS 2 Workspace

`cd /home/ros2_ws`

`colcon build --symlink-install --packages-select px4_msgs`

`source install/setup.bash`

`colcon build --symlink-install --packages-select px4_ros_com`

`source install/setup.bash`

`colcon build`

`source install/setup.bash`


## SSH Key Setup 

Step 1 : Generate a New SSH Key (if none exists)

`ssh-keygen -t ed25519 -C "your_email@example.com"`

Step 2 : Then start the SSH agent and add your key:

eval "$(ssh-agent -s)"

`ssh-add ~/.ssh/id_ed25519`

Step 3 : Add SSH Key to Bitbucket

`cat ~/.ssh/id_ed25519.pub # Copy your public key`

Step 4 : Go to Bitbucket:

Click on your avatar → Personal settings

Go to SSH keys

Click Add key

Paste the public key and save

## Docker Management 

`sudo docker ps -a` # Lists down all current and past dockers

`sudo docker exec -it` # Restart the docker

`docker system prune -a --volumes` # Clean up unused images/containers

`sudo docker rm tii_ros2_agent`# Remove existing docker

`docker stop px4-dev-test` # To stop docker 



 





