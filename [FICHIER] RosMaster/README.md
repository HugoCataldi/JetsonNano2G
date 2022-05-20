# 1) Installation d'Ubuntu 18.04

Toutes les informations liées à l'installation d'ubuntu sont ici : https://releases.ubuntu.com/18.04/

# 2) Installation de ROS Melodic

Toutes les informations liées au projet sont ici : https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup

Résumé des commandes à effectuer :

``` bash
$ sudo apt update
$ sudo apt upgrade
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_melodic.sh
$ chmod 755 ./install_ros_melodic.sh 
$ bash ./install_ros_melodic.sh

$ sudo apt-get install ros-melodic-joy ros-melodic-teleop-twist-joy
$ sudo apt-get install ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc
$ sudo apt-get install ros-melodic-rgbd-launch ros-melodic-depthimage-to-laserscan
$ sudo apt-get install ros-melodic-rosserial-arduino ros-melodic-rosserial-python
$ sudo apt-get install ros-melodic-rosserial-server ros-melodic-rosserial-client
$ sudo apt-get install ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server
$ sudo apt-get install ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro
$ sudo apt-get install ros-melodic-compressed-image-transport ros-melodic-rqt*
$ sudo apt-get install ros-melodic-gmapping ros-melodic-navigation ros-melodic-interactive-markers

$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src/
$ git clone -b melodic-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
$ git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git

$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_manipulation.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_manipulation_simulations.git
$ git clone https://github.com/ROBOTIS-GIT/open_manipulator_dependencies.git
$ sudo apt install ros-melodic-ros-control*
$ sudo apt install ros-melodic-control*
$ sudo apt install ros-melodic-moveit*
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
$ echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
$ source ~/.bashrc
$ cd ~/catkin_ws && catkin_make


$ sudo apt-get install python3-catkin-pkg-modules
$ sudo apt-get install python3-rospkg-modules
```

### Pour configurer ROS, il faut éditer  le fichier bashrc avec la commande suivante :
``` bash
$ nano ~/.bashrc
$ source ~/.bashrc
```

**ROS_MASTER_URI** : L'adresse IP de l'ordinateur qui lance le nœud ROS master

**ROS_HOSTNAME** : L'adresse IP de l'ordinateur qui lance le nœud ROS master

**TURTLEBOT3_MODEL** : Model du turtlebot [ waffle_pi , waffle , burger ]

# 3) Crée un serveur NTP (synchronisation des horloge)

Installez le package ntp et configurez le :
``` bash
sudo apt-get install ntp

sudo nano /etc/ntp.conf
```

Ajoutez les ligne suivante dans le nano :
``` bash
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

Pour relancer le serveur NTP
``` bash
sudo /etc/init.d/ntp restart
```

Pour mettre a jour l'horloge d'un autre appareil (ex: Le turtlebot3):
``` bash
sudo ntp -u 192.168.1.10
```


