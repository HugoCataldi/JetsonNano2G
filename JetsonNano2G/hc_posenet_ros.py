#!/usr/bin/python3


###############################
# Programme   : HcPoseNetRos <- Fait a partir de l'exemple posenet.py du projet github 'jetson-inference'
#
# Description : Programme qui utilise un réseau de neurones et des librairies optimisé pour le jetson
#               permettant de faire de l'estimation de pose.
# 
# Explication : Un squelette est générer avec des points clé sur une personne grace à une IA. 
#               En fonction des angles d'articulation de l'epaule et du coude, une information lié à 
#               la posture est envoyer sur un noeud ROS puis affiché sur la fenetre de la camera.
#               Il y a 4 posture qui renvoi : AVANCER, STOP, GAUCHE, DROITE
#               
# Créateur    : Hugo Cataldi
#
# Date        : 21 / 04 / 2022
#
###############################


########################################################################### Ajouts des librairies

# Ajout des librairies liées au projet jetson-inference
import jetson.inference
import jetson.utils

# Ajout des librairies pour géré les arguments et le lancement du programme
import argparse
import sys
import os

# Ajout des librairies pour calculer les angles
import numpy as np

# Ajout des librairies ROS
import rospy
from std_msgs.msg import String




########################################################################### Initialisation des fonctions

# Gestion des articulations
class art:

    def __init__(self, idx, nom, keypoint):
        # Le nom et l'id de l'articulation
        self.idx = idx
        self.nom = nom
        
        # Les trois points-clés pour calculer l'angle de l'articulation
        self.keypoint = keypoint
        
        # L'angle de l'articulation
        self.angle = -1
        
        # La position dans laquelle se trouve et se trouvait l'articulation
        self.etat = -1
        self.temp_etat = -1
        
        # Si l'état de l'articulation a changé
        self.e = False


    def calc(self, pose):
        # Variable par défaut
        angle = -1
        etat = -1
        e = False
        
        # Recherche des trois points-clés dans la pose
        P1 = pose.FindKeypoint(self.keypoint[0])
        P2 = pose.FindKeypoint(self.keypoint[1])
        P3 = pose.FindKeypoint(self.keypoint[2])

        # Si les trois points-clés sont détectés
        if not (P1 < 0 or P2 < 0 or P3 < 0):
            a = np.array((pose.Keypoints[P1].x,pose.Keypoints[P1].y))
            b = np.array((pose.Keypoints[P2].x,pose.Keypoints[P2].y))
            c = np.array((pose.Keypoints[P3].x,pose.Keypoints[P3].y))
            
            # Calcul de l'angle avec la fonction arc tan
            rad = np.arctan2(c[1]-b[1],c[0]-b[0]) - np.arctan2(a[1]-b[1],a[0]-b[0])
            angle = int(np.abs((rad*180)/np.pi))
            if(angle > 180):
                angle = 360-angle
            
            # Attribution d'un état en fonction d'un intervalle d'angle
            if(0<=angle<60):
                etat=2
            elif(70<=angle<=110):
                etat=1
            elif(150<angle<=180):
                etat=0
             
            # Vérification s'il y a eu un changement d'état
            if(self.temp_etat != etat):
                e = True
                self.temp_etat = etat
                
        # Enregistrement de l'angle, de l'état et du changement d'état
        self.angle = angle
        self.etat = etat
        self.e = e


    # Fonction pour afficher les informations des articulations sur le terminal
    def info(self):
        if(self.etat != -1):
            print("\t{:s} : Angle : {:d}° | Etat : {:d}".format(self.nom,self.angle,self.etat))
        else:
            print("\t{:s} n'est pas totalement detecté(e)".format(self.nom))


# Fonction pour déterminer des postures avec l'état des articulations
def pose_droite(art1, art2, p):
    global msg1
    if(art1.e or art2.e):
        if(art1.etat == 0):
            if(art2.etat == 0):
                msg1 = "DROITE"    #  Ext
                p.publish(msg1)
            elif(art2.etat == 1):
                msg1 = "AVANCER"   # Mil
                p.publish(msg1)
            #elif(art2.etat == 2):
            #    msg1 = "STOP"     # Int
            #    p.publish(msg1)


def pose_gauche(art1, art2, p):
    global msg2
    if(art1.e or art2.e):
        if(art1.etat == 0):
            if(art2.etat == 0):
                msg2 = "GAUCHE"    #  Ext
                p.publish(msg2)
            elif(art2.etat == 1):
                msg2 = "STOP"      # Mil
                p.publish(msg2)
            #elif(art2.etat == 2):
            #    msg2 = "STOP"     # Int
            #    p.publish(msg2)




########################################################################### Lancement du programme
# Gestion des arguments au lancement du programme
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

# Relance le service nvargus pour éviter certaines erreurs
os.system('sudo systemctl restart nvargus-daemon')

# Charge le modèle de réseaux neurones qui permet d'estimer la pose
net = jetson.inference.poseNet(opt.network, sys.argv, opt.threshold)

# Crée les sources d'entrée et de sortie
input = jetson.utils.videoSource(opt.input_URI, argv=sys.argv)
output = jetson.utils.videoOutput(opt.output_URI, argv=sys.argv)

# Initialisation de ROS
rospy.init_node('listener')
p = rospy.Publisher('jetcam', String, queue_size=10)

# Enregistrement des articulations
COUDE_GAUCHE = art(0,"COUDE_GAUCHE", ('left_shoulder', 'left_elbow', 'left_wrist'))
COUDE_DROIT = art(1,"COUDE_DROIT", ('right_shoulder', 'right_elbow', 'right_wrist'))
EPAULE_GAUCHE = art(2,"EPAULE_GAUCHE", ('neck', 'left_shoulder', 'left_elbow'))
EPAULE_DROITE = art(3,"EPAULE_DROITE", ('neck', 'right_shoulder', 'right_elbow'))

# Variable pour afficher la position des bras sur la fenêtre
msg1="none"
msg2="none"




########################################################################### Boucle principale qui gère le traitement des images
while True:
    # Recupère les images de la caméra
    img = input.Capture()

    # Processus qui permet l'estimation de la pose
    poses = net.Process(img, overlay=opt.overlay)


    # Affiche le nombre d'objets détectés
    print("======================================================")
    print("Nombre d'objects detectés : {:d}".format(len(poses)))

    # Boucle for pour le traitement des informatation de chaque pose/personne détectée
    for (idx,pose) in enumerate(poses):
        print("Personne n°{:d} :".format(idx+1))

        COUDE_GAUCHE.calc(pose)
        COUDE_GAUCHE.info()
        EPAULE_GAUCHE.calc(pose)
        EPAULE_GAUCHE.info()

        COUDE_DROIT.calc(pose)
        COUDE_DROIT.info()
        EPAULE_DROITE.calc(pose)
        EPAULE_DROITE.info()

        if(idx == 0):

            pose_gauche(EPAULE_GAUCHE,COUDE_GAUCHE,p)
            pose_droite(EPAULE_DROITE,COUDE_DROIT,p)

    # Affiche le resultat de la camera a l'ecran
    output.Render(img)

    # Affiche le réseau de neurones utilisé, le nombre de FPS et les positions sur la barre de titre de la fenêtre de sortie
    output.SetStatus("Reseau de neuronnes : {:s} | FPS : {:.0f} | Bras droit : {:s} | Bras gauche : {:s}".format(opt.network, net.GetNetworkFPS(), msg1,msg2))

    # Permets de quitter la boucle infinie
    if not input.IsStreaming() or not output.IsStreaming():
        break
