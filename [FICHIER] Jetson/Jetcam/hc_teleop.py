#!/usr/bin/python3


###############################
# Programme   : 
#
# Description : 
#
# Explication : 
#
# Créateur    : Hugo Cataldi
#
# Date        :  /  / 2022
#
###############################


# Ajout des librairies pour géré les arguments et le lancement du programme
import argparse
import os
import sys

# Ajout des librairies liées au projet jetson-inference
import jetson.inference
import jetson.utils

# Ajout du module pour calculer les angles
from modules.HcArticulation import *

# Relance le service nvargus pour éviter certaines erreurs
os.system('sudo systemctl restart nvargus-daemon')


#  Adresse IP du flux RTP
ip = str(input("Adresse IP pour la sortie video en rtp : "))
if ip == "":
    stream_ip = "rtp://192.168.1.10:1234"
else:
    stream_ip = "rtp://{:s}:1234".format()


# Ajout des librairies ROS et du module pour contrôler le robot
mode = str(input("Pour lancer le programme sans ROS, entrer B ou b : "))
if mode != "B" or mode != "b":
    import rospy
    from std_msgs.msg import String
    from modules.HcRobot import *

    rospy.init_node('jetcam_teleop')

    p = rospy.Publisher('jetcam', String, queue_size=10)

    turtlebot3 = robot(0, "waffle")




# Analyse des arguments de la ligne de commande
parser = argparse.ArgumentParser(description="Estimation de pose DNN sur une image/vidéo.",
                                 formatter_class=argparse.RawTextHelpFormatter,
                                 epilog=jetson.inference.poseNet.Usage() +
                                 jetson.utils.videoSource.Usage() +
                                 jetson.utils.videoOutput.Usage() +
                                 jetson.utils.logUsage())
# URI de l'entrée
parser.add_argument("input_URI", type=str,default="csi://0",nargs="?")
# URI de la sortie
parser.add_argument("output_URI", type=str,default=stream_ip,nargs="?")
# Choix du modèle pre-entrainer
parser.add_argument("--network", type=str,default="resnet18-body",nargs="?")
# Objets à afficher sur l'overlay  : 'links', 'keypoints', 'boxes', 'none'
parser.add_argument("--overlay", type=str,default="keypoints",nargs="?")
# Seuil de détection minimum
parser.add_argument("--threshold", type=float, default=0.15,nargs="?")

try:
    opt = parser.parse_known_args()[0]
except:
    print("")
    parser.print_help()
    sys.exit(0)



# Charge le modèle de réseaux neurones qui permet d'estimer la pose
net = jetson.inference.poseNet(opt.network, sys.argv, opt.threshold)

# Crée les sources d'entrée et de sortie
input = jetson.utils.videoSource(opt.input_URI, argv=sys.argv)
output = jetson.utils.videoOutput(opt.output_URI, argv=sys.argv)


# Variable pour afficher la position des bras sur la fenêtre
msgs = "NONE"
tmsgs = "NONE"
modeAuto = False

# Boucle principale qui gère le traitement des images
while True:
    # Récupère les images de la caméra
    img = input.Capture()

    # Processus qui permet l'estimation de la pose
    poses = net.Process(img, overlay=opt.overlay)

    # Affiche le nombre d'objets détectés
    print("\n\n\n======================================================")
    print("Nombre d'objects detectés : {:d}".format(len(poses)))

    # Boucle for pour le traitement des informations de chaque pose/personne détectée
    for (idx, pose) in enumerate(poses):
        print("Personne n°{:d} :".format(idx+1))

        if(idx == 0):
            # Actualisation des angles
            COUDE_GAUCHE.calc(pose)
            EPAULE_GAUCHE.calc(pose)
            COUDE_DROIT.calc(pose)
            EPAULE_DROITE.calc(pose)

            # Info des articulations
            COUDE_GAUCHE.info()
            EPAULE_GAUCHE.info()
            COUDE_DROIT.info()
            EPAULE_DROITE.info()

            # Détermine une commande en fonction de la posture
            msgs = pose_gauche(EPAULE_GAUCHE, COUDE_GAUCHE, msgs)
            msgs = pose_droite(EPAULE_DROITE, COUDE_DROIT, msgs)
            print("\n\tMode de fonctionnement :", msgs)

    # Si ROS est actif
    if ((mode != "B" or mode != "b") and (msgs!="NONE")):
        if(len(poses) == 0):
            msgs = "NONE"
            turtlebot3.STOP()
        else:
            if(tmsgs != msgs):
                tmsgs = msgs
                p.publish(msgs)
                modeAuto = False

                if(msgs == "AVANCER"):
                    turtlebot3.STOP()
                    turtlebot3.AVANCER()

                elif(msgs == "DROITE"):
                    turtlebot3.STOP()
                    turtlebot3.DROITE()

                elif(msgs == "GAUCHE"):
                    turtlebot3.STOP()
                    turtlebot3.GAUCHE()

                elif(msgs == "STOP"):
                    turtlebot3.STOP()
                
                elif(msgs == "TRAK"):
                    modeAuto = True
            
            
            if(modeAuto):
                tL, tR = trak(pose, 'neck')
                turtlebot3.STOP()
                if(tL == 1):
                    turtlebot3.AVANCER()
                elif(tL == 2):
                    turtlebot3.RECULER()
                if(tR == 1):
                    turtlebot3.DROITE()
                elif(tR == 2):
                    turtlebot3.GAUCHE()
        

        turtlebot3.GO()        

    # Affiche le résultat de la caméra a l'écran
    output.Render(img)

    # Permets de quitter la boucle infinie
    if not input.IsStreaming() or not output.IsStreaming():
        break
