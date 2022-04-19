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

$ sudo apt-get install ros-kinetic-dynamixel-sdk
$ sudo apt-get install ros-kinetic-turtlebot3-msgs

$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src/
$ git clone -b kinetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
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
```

### Pour configurer ROS, il faut éditer  le fichier bashrc avec la commande suivante :
``` bash
$ nano ~/.bashrc
$ source ~/.bashrc
```
<img src="images/bashrc.PNG" width="650">

**ROS_MASTER_URI** : L'adresse IP de l'ordinateur qui lance le nœud ROS master

**ROS_HOSTNAME** : L'adresse IP de l'ordinateur qui lance le nœud ROS master

**TURTLEBOT3_MODEL** : Model du turtlebot [ waffle_pi , waffle , burger ]

