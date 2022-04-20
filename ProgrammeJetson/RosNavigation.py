#!/usr/bin/python3


######################################## Ajouts des librairies ########################################
import jetson.inference
import jetson.utils

import argparse
import sys

# Ajout des librairies pour calculer les angles
import numpy as np

# Ajout des librairies ROS
import rospy
from std_msgs.msg import String




######################################## Initialisation des fonctions ########################################
# Récupération des argument de la ligne de commande
def init_arg():
    parser = argparse.ArgumentParser(description="Lancement de l'estimation de poses DNN sur un stream video", 
                                     formatter_class=argparse.RawTextHelpFormatter,
                                     epilog=jetson.inference.poseNet.Usage() +
                                     jetson.utils.videoSource.Usage() +
                                     jetson.utils.videoOutput.Usage() +
                                     jetson.utils.logUsage())

    parser.add_argument("input_URI", type=str, default="csi://0", nargs='?', help="Permet de choisir le type de camera utilisé (csi://0 <- Raspicam V2.1)")
    parser.add_argument("output_URI", type=str, default="", nargs='?', help="Permet de rediriger le flux video sur une autre source, par exemple en rtp")
    parser.add_argument("--network", type=str, default="resnet18-body", help="Permet de choisir un modèle pré-entrainer (resnet18-hand-pose <- pour les main)")
    parser.add_argument("--overlay", type=str, default="links,keypoints", help="Permet de choisir ce qui sera afficher sur la sortie video : 'links', 'keypoints', 'boxes', 'none'")
    parser.add_argument("--threshold", type=float, default=0.10, help="Seuil de détéction minimum") 

    try:
        opt = parser.parse_known_args()[0]
    except:
        print("")
        parser.print_help()
        sys.exit(0)

# Calcul des angles
class art:

    def __init__(self, idx, nom, keypoint):
        self.idx = idx
        self.nom = nom
        self.keypoint = keypoint
        self.angle = -1
        self.etat = -1
        self.temp_etat = -1


    def calc(self, pose):
        # Calcul de l'angle
        P1 = pose.FindKeypoint(self.keypoint[0])
        P2 = pose.FindKeypoint(self.keypoint[1])
        P3 = pose.FindKeypoint(self.keypoint[2])
        angle = -1
        etat = -1
        if not (P1 < 0 or P2 < 0 or P3 < 0):
            a = np.array((pose.Keypoints[P1].x,pose.Keypoints[P1].y))
            b = np.array((pose.Keypoints[P2].x,pose.Keypoints[P2].y))
            c = np.array((pose.Keypoints[P3].x,pose.Keypoints[P3].y))
            rad = np.arctan2(c[1]-b[1],c[0]-b[0]) - np.arctan2(a[1]-b[1],a[0]-b[0])
            angle = int(np.abs((rad*180)/np.pi))
            etat = int(angle/72)
        self.etat = etat
        self.angle = angle

        if((self.temp_etat != self.etat) and self.etat!=-1):
            self.temp_etat = self.etat
            return etat
        return 0

    def info(self):
        if(self.etat != -1):
            print("\t{:s} : Angle : {:d}° | Etat : {:d}".format(self.nom,self.angle,self.etat))
        else:
            print("\t{:s} n'est pas totalement detecté(e)".format(self.nom))

def actualisation_pose(pose):

# Calcul des positions
def info_pose(e1, e2, p):
    if(e1 != 0 and e1 != 0):
        if(e1 == 2):
            if(e1 == 0):
                p.publish("GAUCHE")
            elif(e1 == 1):
                p.publish("AVANCER")
            elif(e1 == 2):
                p.publish("DROITE")            






######################################### Lancement du programme #############################################
# Argument pour la ligne de commande
init_arg()

# Charge le model de neuronnes qui permet d'estimer la pose
net = jetson.inference.poseNet(opt.network, sys.argv, opt.threshold)

# Crée les sources d'entrée et de sortie
input = jetson.utils.videoSource(opt.input_URI, argv=sys.argv)
output = jetson.utils.videoOutput(opt.output_URI, argv=sys.argv)

# Initialisation de ROS
rospy.init_node('listener')
p = rospy.Publisher('jetcam', String, queue_size=10)

# Enregistrement des articulation
COUDE_GAUCHE = art(0,"COUDE_GAUCHE", ('left_shoulder', 'left_elbow', 'left_wrist'))
COUDE_DROIT = art(1,"COUDE_DROIT", ('right_shoulder', 'right_elbow', 'right_wrist'))
EPAULE_GAUCHE = art(2,"EPAULE_GAUCHE", ('neck', 'left_shoulder', 'left_elbow'))
EPAULE_DROITE = art(3,"EPAULE_DROITE", ('neck', 'right_shoulder', 'right_elbow'))





################################## Boucle principale qui gère le traitement des image ####################################################
while True:
    # Recupere les image de la camera
    img = input.Capture()

    # Processus qui permet l'estimation de la pose
    poses = net.Process(img, overlay=opt.overlay)


    print("======================================================")
    # Affiche le nombre de poses detectés
    print("Nombre d'objects detectés : {:d}".format(len(poses)))

    # Liste les information de chaque poses detectés
    for (idx,pose) in enumerate(poses):
        print("Personne n°{:d} :".format(idx+1))

        a1 = COUDE_GAUCHE.calc(pose)
        a2 = EPAULE_GAUCHE.calc(pose)

        b1 = COUDE_DROIT.calc(pose)
        b2 = EPAULE_DROITE.calc(pose)


        if(idx == 0):
            info_pose(a1,a2, p)
            print(a1)
            print(a2)
            print(COUDE_GAUCHE.angle)

    print("======================================================")

    # Affiche le resultat de la camera a l'ecran
    output.Render(img)

    # Affiche le reseau de neronnes utiliser et le nombre de FPS sur la barre de titre de la fenetre de la camera
    output.SetStatus("Reseau de neuronnes : {:s} | FPS : {:.0f}".format(opt.network, net.GetNetworkFPS()))

    # Affiche les différente information sur les performance dans la console
    net.PrintProfilerTimes()

    # Permet de quitter la boucle infinie
    if not input.IsStreaming() or not output.IsStreaming():
        break
