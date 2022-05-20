#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist


BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.15
ANG_VEL_STEP_SIZE = 0.9

# ########################################### #
#     Gestion de la trajectoire du robot      #
# ########################################### #
class robot:

    def __init__(self, idx, model):
        # L'ID et le modèle du robot
        self.idx = idx
        self.model = rospy.get_param("model", model)

        # Initialisation du publisher pour envoyer les requêtes au robot
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # Variables des vélocités
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0
        self.control_linear_vel = 0.0
        self.control_angular_vel = 0.0
        self.temp_tlv = 0.0
        self.temp_tav = 0.0

    # Reset la vélocité linéaire
    def rL(self):
        self.target_linear_vel = 0.0
        self.control_linear_vel = 0.0

    # Reset la vélocité angulaire
    def rR(self):
        self.target_angular_vel = 0.0
        self.control_angular_vel = 0.0

    # Incrémente la vélocité linéaire d'un pas
    def AVANCER(self):
        self.rL()
        self.target_linear_vel = checkLinearLimitVelocity(
            self.target_linear_vel + LIN_VEL_STEP_SIZE, self.model)

    # Décrémente la vélocité linéaire d'un pas
    def RECULER(self):
        self.rL()
        self.target_linear_vel = checkLinearLimitVelocity(
            self.target_linear_vel - LIN_VEL_STEP_SIZE, self.model)

    # Incrémente la vélocité angulaire d'un pas
    def DROITE(self):
        self.rR()
        self.target_angular_vel = checkAngularLimitVelocity(
            self.target_angular_vel + ANG_VEL_STEP_SIZE, self.model)

    # Décrémente la vélocité angulaire d'un pas
    def GAUCHE(self):
        self.rR()
        self.target_angular_vel = checkAngularLimitVelocity(
            self.target_angular_vel - ANG_VEL_STEP_SIZE, self.model)

    # Reset toutes les vélocités
    def STOP(self):
        self.rL()
        self.rR()

    # Envoie une requête au robot comportant les nouvelles vélocités
    def GO(self):
        if(self.temp_tlv != self.target_linear_vel or self.temp_tav != self.target_angular_vel): # Condition de changement de vélocité
            self.temp_tlv = self.target_linear_vel
            self.temp_tav = self.target_angular_vel

            twist = Twist()

            self.control_linear_vel = makeSimpleProfile(
                self.control_linear_vel, self.target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
            twist.linear.x = self.control_linear_vel
            twist.linear.y = 0.0
            twist.linear.z = 0.0

            self.control_angular_vel = makeSimpleProfile(
                self.control_angular_vel, self.target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = self.control_angular_vel

            # Envoi de la requête avec un publisher
            self.publisher.publish(twist)


# #################################################################################### #
#        Fonction pour la vérification des différentes vélocités et contraintes        #
# #################################################################################### #
def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel, target_angular_vel)


def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
    else:
        output = input
    return output


def constrain(input, low, high):
    if input < low:
        input = low
    elif input > high:
        input = high
    else:
        input = input
    return input


def checkLinearLimitVelocity(vel, turtlebot3_model):
    if turtlebot3_model == "burger":
        vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
        vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
    else:
        vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    return vel


def checkAngularLimitVelocity(vel, turtlebot3_model):
    if turtlebot3_model == "burger":
        vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
        vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
    else:
        vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    return vel
