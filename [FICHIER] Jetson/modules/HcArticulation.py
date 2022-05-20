#!/usr/bin/python3


# Ajout des librairies pour calculer les angles
import numpy as np


# Gestion des articulations
class art:

    def __init__(self, nom, keypoint):
        # Le nom de l'articulation
        self.nom = nom
        # Les trois points-clés pour calculer l'angle de l'articulation
        self.keypoint = keypoint
        # L'angle de l'articulation
        self.A = -1
        # La position dans laquelle se trouve et se trouvait l'articulation
        self.E = -1
        self.tE = -1
        # Si l'état de l'articulation a changé
        self.event = False

    def calc(self, pose):
        # Variable par défaut
        A = -1
        E = -1
        event = False

        # Recherche des trois points-clés dans la pose
        P1 = pose.FindKeypoint(self.keypoint[0])
        P2 = pose.FindKeypoint(self.keypoint[1])
        P3 = pose.FindKeypoint(self.keypoint[2])

        # Si les trois points-clés sont détectés
        if not (P1 < 0 or P2 < 0 or P3 < 0):
            a = np.array((pose.Keypoints[P1].x, pose.Keypoints[P1].y))
            b = np.array((pose.Keypoints[P2].x, pose.Keypoints[P2].y))
            c = np.array((pose.Keypoints[P3].x, pose.Keypoints[P3].y))

            # Calcul de l'angle avec la fonction arc tan
            rad = np.arctan2(c[1]-b[1], c[0]-b[0]) - \
                np.arctan2(a[1]-b[1], a[0]-b[0])
            A = int(np.abs((rad*180)/np.pi))
            if(A > 180):
                A = 360-A

            # Attribution d'un état en fonction d'un intervalle d'angle
            if(0 <= A <= 40):
                E = 2
            elif(60 <= A <= 130):
                E = 1
            elif(150 <= A <= 180):
                E = 0

            # Vérification s'il y a eu un changement d'état
            if(self.tE != E):
                event = True
                self.tE = E

        # Enregistrement de l'angle, de l'état et du changement d'état
        self.A = A
        self.E = E
        self.event = event

    # Fonction pour afficher les informations des articulations sur le terminal
    def info(self):
        if(self.E != -1):
            print("\t{:s} : Angle : {:d}° | Etat : {:d}".format(
                self.nom, self.A, self.E))
        else:
            print("\t{:s} n'est pas totalement detecté(e)".format(self.nom))

# Enregistrement des articulations
COUDE_GAUCHE = art("COUDE_GAUCHE", ('left_shoulder', 'left_elbow', 'left_wrist'))
COUDE_DROIT = art("COUDE_DROIT", ('right_shoulder', 'right_elbow', 'right_wrist'))
EPAULE_GAUCHE = art("EPAULE_GAUCHE", ('neck', 'left_shoulder', 'left_elbow'))
EPAULE_DROITE = art("EPAULE_DROITE", ('neck', 'right_shoulder', 'right_elbow'))


# ######################################################################### #
#    Fonction pour déterminer des postures avec l'état des articulations    #
# ######################################################################### #
def pose_droite(art1, art2, msgs):
    if(art1.event or art2.event):
        if(art1.E == 0):
            if(art2.E == 0):      # Ext
                return "DROITE"
            elif(art2.E == 1):    # Mil
                return "AVANCER"
            elif(art2.E == 2):    # Int
                return "NONE"
    return msgs


def pose_gauche(art1, art2, msgs):
    if(art1.event or art2.event):
        if(art1.E == 0):
            if(art2.E == 0):      # Ext
                return "GAUCHE"
            elif(art2.E == 1):    # Mil
                return "STOP"
            elif(art2.E == 2):    # Int
                return "TRAK"
    return msgs

# ######################################################## #
#    Fonction qui permet de traquer un point de la pose    #
# ######################################################## #
def trak(pose,keypoint):
    n = pose.FindKeypoint(keypoint)
    tL = 0
    tR = 0
    if not (n < 0): # Si le point est détecté
        x = pose.Keypoints[n].x
        if(x <= 512): # Trop à gauche
            tR = 1
        elif(x >= 778): # Trop à droite
            tR = 2

        y = pose.Keypoints[n].y
        if(y <= 282): # Trop bas
            tL = 2
        elif(y >= 438): # Trop haut
            tL = 1

    return tL, tR



