#!/usr/bin/env python

import sys
import copy
import rospy

from std_msgs.msg import String

import moveit_commander
import moveit_msgs.msg


from math import pi   # Importer le nombre pi


# Fonction pour convertir les radian et les degree
def convRad(deg):
    rad = deg * ((pi)/180)
    return rad


def convDeg(rad):
    deg = rad * (180/(pi))
    return deg


# Fonction qui calcule la marge d'erreur des articulation pour empecher le crash du bras robotique
def compare(joint_group, joint_target):
    for (idx, joint) in enumerate(joint_group):
        if(joint_target[idx] != 0.0):
            erreur = ((joint-joint_target[idx])/(joint_target[idx]))*100
        else:
            erreur = (
                ((joint+1)-(joint_target[idx]+1))/(joint_target[idx]+1))*100
        print(erreur)
        if (abs(erreur) > 25):
            return True
    return False


class openmanipulatorx(object):
    """openmanipulatorx"""

    def __init__(self):
        super(openmanipulatorx, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("openmanipulatorx")

        robot = moveit_commander.RobotCommander()

        # Variable des info du bras
        arm_group = moveit_commander.MoveGroupCommander("arm")
        arm_joint = arm_group.get_current_joint_values()

        grip_group = moveit_commander.MoveGroupCommander("gripper")
        grip_joint = grip_group.get_current_joint_values()

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.arm_group = arm_group
        self.arm_joint = arm_joint
        self.arm_joint_target = []

        self.grip_group = grip_group
        self.grip_joint = grip_joint
        self.grip_joint_target = []

        self.rotate_state = 0
        self.pose_state = 0
        self.grip_state = 0

    # Fonction qui permet d ouvrir et de fermer le gripper

    def cmdGrip(self, num):
        if(num != self.grip_state):
            # Recuperation des angle d articulation
            self.grip_joint = self.grip_group.get_current_joint_values()
            self.grip_joint_target = []
            for joint in self.grip_joint:
                self.grip_joint_target.append(joint)

            # Fermer
            if(num == 0):
                self.grip_joint_target[0] = 0.01
                self.grip_joint_target[1] = 0.01
                self.grip_state = 0
            # Ouvert
            else:
                self.grip_joint_target[0] = -0.01
                self.grip_joint_target[1] = -0.01
                self.grip_state = 1

            # Envoi d une requete pour effetuer le mouvement
            if(compare(self.grip_joint, self.grip_joint_target)):
                self.grip_group.go(self.grip_joint_target, wait=True)
                self.grip_group.stop()
                self.grip_group.clear_pose_targets()

    def pose(self, num):
        if(num != self.pose_state):
            self.arm_joint = self.arm_group.get_current_joint_values()
            self.arm_joint_target = []
            for joint in self.arm_joint:
                self.arm_joint_target.append(joint)

            if(num == 0):
                self.pose_state = 0
                self.arm_joint_target[1] = convRad(-90)
                self.arm_joint_target[2] = convRad(70)
                self.arm_joint_target[3] = convRad(35)
            # Position Haute
            elif(num == 1):
                self.pose_state = 1
                self.arm_joint_target[1] = 0
                self.arm_joint_target[2] = 0
                self.arm_joint_target[3] = 0
            # Position Milieu Haut
            elif(num == 2):
                self.pose_state = 2
                self.arm_joint_target[1] = convRad(-55)
                self.arm_joint_target[2] = convRad(17)
                self.arm_joint_target[3] = convRad(38)
            # Position Milieu Bas
            elif(num == 3):
                self.pose_state = 3
                self.arm_joint_target[1] = convRad(45)
                self.arm_joint_target[2] = 0
                self.arm_joint_target[3] = convRad(-45)
            # Position Basse
            elif(num == 4):
                self.pose_state = 4
                self.arm_joint_target[1] = convRad(75)
                self.arm_joint_target[2] = convRad(-25)
                self.arm_joint_target[3] = convRad(-55)

            self.armGo(self.arm_joint, self.arm_joint_target)

    def rotate(self, num):
        # Arriere Gauche
        if(num != self.rotate_state):

            self.arm_joint = self.arm_group.get_current_joint_values()
            self.arm_joint_target = []
            for joint in self.arm_joint:
                self.arm_joint_target.append(joint)

            # Arriere Gauche
            if(num == -3):
                self.rotate_state = -3
                self.arm_joint_target[0] = convRad(135)
            # Gauche
            elif(num == -2):
                self.rotate_state = -2
                self.arm_joint_target[0] = convRad(90)
            # Devant Gauche
            elif(num == -1):
                self.rotate_state = -1
                self.arm_joint_target[0] = convRad(45)

            # Devant
            elif(num == 0):
                self.rotate_state = 0
                self.arm_joint_target[0] = 0

            # Devant droite
            elif(num == 1):
                self.rotate_state = 1
                self.arm_joint_target[0] = convRad(-45)
            # Droite
            elif(num == 2):
                self.rotate_state = 2
                self.arm_joint_target[0] = convRad(-90)
            # Arriere Droit
            elif(num == 3):
                self.rotate_state = 3
                self.arm_joint_target[0] = convRad(-135)

            self.armGo(self.arm_joint, self.arm_joint_target)

    # Fonction qui permet de remettre le bras dans sa position initiale
    def reset(self):
        if(self.pose_state != 0 or self.rotate_state != 0):
            self.arm_joint = self.arm_group.get_current_joint_values()
            self.arm_joint_target = []
            for joint in self.arm_joint:
                self.arm_joint_target.append(joint)

            self.arm_joint_target[0] = 0
            self.arm_joint_target[1] = convRad(-90)
            self.arm_joint_target[2] = convRad(70)
            self.arm_joint_target[3] = convRad(35)

            self.armGo(self.arm_joint, self.arm_joint_target)

    # Fonction qui envoi une requete au bras pour effecter un mouvement
    def armGo(self, arm_joint, arm_joint_target):
        if(compare(arm_joint, arm_joint_target)):
            self.arm_group.go(arm_joint_target, wait=True)
            self.arm_group.stop()
            self.arm_group.clear_pose_targets()


def callback(cmd):
    global omx
    x = cmd.data.split(":")
    if(x[0] == "reset"):
        omx.reset()
        rospy.sleep(4)
    elif(x[0] == "rotate"):
        omx.rotate(int(x[1]))
        rospy.sleep(4)
    elif(x[0] == "pose"):
        omx.pose(int(x[1]))
        rospy.sleep(4)

    elif(x[0] == "gripper"):
        omx.cmdGrip(int(x[1]))


if __name__ == '__main__':
    omx = openmanipulatorx()

    # Subscriber -> Listener -> Recuperation d'information
    rospy.Subscriber('omx_cmd', String, callback)

    # Pour ne pas que le programme python se s'arrete avant le noeud
    rospy.spin()
