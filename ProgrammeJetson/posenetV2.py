#!/usr/bin/python3
#
# Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.
#

import jetson.inference
import jetson.utils

import argparse
import sys

########################### Fonction pour calculer les angles des articulations ##############################
import numpy as np

def info_articulation(pose,p1,p2,p3):
    P1 = pose.FindKeypoint(p1)
    P2 = pose.FindKeypoint(p2)
    P3 = pose.FindKeypoint(p3)
    
    etat=0
    angle = -1
    if not (P1 < 0 or P2 < 0 or P3 < 0):
        a = np.array((pose.Keypoints[P1].x,pose.Keypoints[P1].y))
        b = np.array((pose.Keypoints[P2].x,pose.Keypoints[P2].y))
        c = np.array((pose.Keypoints[P3].x,pose.Keypoints[P3].y))

        rad = np.arctan2(c[1]-b[1],c[0]-b[0]) - np.arctan2(a[1]-b[1],a[0]-b[0])
        angle = int(np.abs((rad*180)/np.pi))
        if(angle > 180):
            angle = 360-angle

        if(0<=angle<50):
            etat=2
        elif(50<=angle<=130):
            etat=1
        elif(130<angle<=180):
            etat=0

    return angle,etat

########################### Fonction afficher les information des articulation ##############################
def affich_articulation(nom,artic):
    if(artic[0] > 0):
        if(artic[1]==2):
            etat="Flechis"
        elif(artic[1]==1):
            etat="Tendu"
        elif(artic[1]==0):
            etat="Repos"
        print("\t{:s} : Angle : {:.2f}° | Etat : {:s}".format(nom,artic[0],etat))

    else:
        print("\t{:s} n'est pas totalement detecté(e)".format(nom))

##########################################################################################################


# Argument pour la ligne de commande
parser = argparse.ArgumentParser(description="Run pose estimation DNN on a video/image stream.", 
                                 formatter_class=argparse.RawTextHelpFormatter, epilog=jetson.inference.poseNet.Usage() +
                                 jetson.utils.videoSource.Usage() + jetson.utils.videoOutput.Usage() + jetson.utils.logUsage())

parser.add_argument("input_URI", type=str, default="", nargs='?', help="URI of the input stream")
parser.add_argument("output_URI", type=str, default="", nargs='?', help="URI of the output stream")
parser.add_argument("--network", type=str, default="resnet18-body", help="pre-trained model to load (see below for options)")
parser.add_argument("--overlay", type=str, default="links,keypoints", help="pose overlay flags (e.g. --overlay=links,keypoints)\nvalid combinations are:  'links', 'keypoints', 'boxes', 'none'")
parser.add_argument("--threshold", type=float, default=0.15, help="minimum detection threshold to use") 

try:
	opt = parser.parse_known_args()[0]
except:
	print("")
	parser.print_help()
	sys.exit(0)

# Charge le model de neuronnes qui permet d'estimer la pose
net = jetson.inference.poseNet(opt.network, sys.argv, opt.threshold)

# Crée les sources d'entrée et de sortie
input = jetson.utils.videoSource(opt.input_URI, argv=sys.argv)
output = jetson.utils.videoOutput(opt.output_URI, argv=sys.argv)

# Boucles qui gère le traitement des image
while True:
    # Recupere les image de la camera
    img = input.Capture()

    # Processus qui permet l'estimation de la pose
    poses = net.Process(img, overlay=opt.overlay)

    ###################################################################################################################################################
    print("======================================================")
    # Affiche le nombre de poses detectés
    print("Nombre d'objects detectés : {:d}".format(len(poses)))

    # Liste les information de chaque poses detectés
    for (idx,posex) in enumerate(poses):
        print("Personne n°{:d} :".format(idx))

        Bras_Gauche = info_articulation(posex, 'left_shoulder', 'left_elbow', 'left_wrist')
        affich_articulation("Bras gauche",Bras_Gauche)

        Bras_Droit = info_articulation(posex, 'right_shoulder', 'right_elbow', 'right_wrist')
        affich_articulation("Bras droit",Bras_Droit)


        Epaule_Gauche = info_articulation(posex, 'neck', 'left_shoulder', 'left_elbow')
        affich_articulation("Epaule_Gauche",Epaule_Gauche)

        Epaule_Droite = info_articulation(posex, 'neck', 'right_shoulder', 'right_elbow')
        affich_articulation("Epaule_Droite",Epaule_Droite)

    print("======================================================") ####################################################################################################################################################

    # Affiche le resultat de la camera a l'ecran
    output.Render(img)

    # Affiche le reseau de neronnes utiliser et le nombre de FPS sur la barre de titre de la fenetre de la camera
    output.SetStatus("Reseau de neuronnes : {:s} | FPS : {:.0f}".format(opt.network, net.GetNetworkFPS()))

    # Affiche les différente information sur les performance dans la console
    net.PrintProfilerTimes()

    # Permet de quitter la boucle infinie
    if not input.IsStreaming() or not output.IsStreaming():
        break
