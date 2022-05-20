# Installation d'Ubuntu 20.04

Toutes les informations liées à l'installation d'ubuntu sont ici : https://releases.ubuntu.com/20.04/

# Installation de ROS Noetic

Toutes les informations liées au projet sont ici : https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup

Résumé des commandes à effectuer :

``` bash
$ sudo apt update
$ sudo apt upgrade
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_noetic.sh
$ chmod 755 ./install_ros_noetic.sh 
$ bash ./install_ros_noetic.sh

$ sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy
$ sudo apt-get install ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc
$ sudo apt-get install ros-noetic-rgbd-launch ros-noetic-rosserial-arduino
$ sudo apt-get install ros-noetic-rosserial-python ros-noetic-rosserial-client
$ sudo apt-get install ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server
$ sudo apt-get install ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro
$ sudo apt-get install ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz
$ sudo apt-get install ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers

$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src/
$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git

$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_manipulation.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_manipulation_simulations.git
$ git clone https://github.com/ROBOTIS-GIT/open_manipulator_dependencies.git
$ sudo apt install ros-noetic-ros-control*
$ sudo apt install ros-noetic-control*
$ sudo apt install ros-noetic-moveit*
$ cd ~/catkin_ws && catkin_make
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

$ echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
$ source ~/.bashrc

$ sudo apt-get install python3-catkin-pkg-modules
$ sudo apt-get install python3-rospkg-modules









sudo /etc/init.d/ntp stop
sudo ntpd -q 192.168.1.10
sudo ntpdate 192.168.1.10



 gst-launch-1.0 -v udpsrc port=1234 \
 caps = "application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96" ! \
 rtph264depay ! decodebin ! videoconvert ! autovideosink






driftfile /var/lib/ntp/drift
broadcastdelay 0.008

# Give localhost full access rights
restrict 127.0.0.1

# Give machines on our network access to query us
restrict 192.168.1.0 mask 255.255.255.0 nomodify notrap

broadcast 192.168.1.0

server 127.127.1.0 prefer
fudge  127.127.1.0 stratum 10
```

### Pour configurer ROS, il faut éditer  le fichier bashrc avec la commande suivante :
``` bash
$ nano ~/.bashrc
$ source ~/.bashrc
```

**ROS_MASTER_URI** : L'adresse IP de l'ordinateur qui lance le nœud ROS master

**ROS_HOSTNAME** : L'adresse IP de l'ordinateur qui lance le nœud ROS master

**TURTLEBOT3_MODEL** : Model du turtlebot [ waffle_pi , waffle , burger ]
