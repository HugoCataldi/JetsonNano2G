
# Ajout des librairies pour calculer les angles
import numpy as np

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