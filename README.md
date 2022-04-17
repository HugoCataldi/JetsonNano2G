# Installation des outils pour utiliser le projet "jetson-inference"

Toute les information liées au projet son ici : https://github.com/dusty-nv/jetson-inference


## Etape 1 : Téléchargement du projet

Pour télécharger le code, assurez-vous que git et cmake sont installés.

``` bash
$ sudo apt-get update
$ sudo apt-get install git cmake
```

Après on clone le projet `jetson-inference` :

``` bash
$ git clone https://github.com/dusty-nv/jetson-inference
$ cd jetson-inference
$ git submodule update --init
```


##  Etape 2 : Paquets de développement Python

Le projet a besoin des modules d'extension Python qui fournissent des liaisons au code C++ à l'aide de l'API Python C.

Donc, si vous voulez que le projet crée des liaisons pour Python 3.6, installez ces packages avant de continuer :

``` bash
$ sudo apt-get install libpython3-dev python3-numpy
``` 


## Configuration avec la commande CMake

Ensuite, créez un répertoire pour construire le projet et exécutez la commande `cmake`  . Lorsque `cmake` est exécuté, un script est lancé (`CMakePreBuild.sh`) qui installera toutes les dépendances requises et téléchargera les modèles DNN(Deep Neural Network) pour vous.

``` bash
$ mkdir build
$ cd build
$ cmake ../
```


## Téléchargement des modèles

Le projet est livré avec de nombreux réseaux déja entrainer que vous pouvez choisir de télécharger et d'installer via l' outil Model Downloader.

Le menu suivant s'affichera, appuyez sur la touche `espace` pour séléctionner les réseau à télécharger. Appuyez sur `entrer` pour continuer.


<img src="https://raw.githubusercontent.com/dusty-nv/jetson-inference/python/docs/images/download-models.jpg" width="650">

Si vous voulez relancer l'**outil de téléchargement de modèles** plus tard, utilisez la commande suivante :

``` bash
$ cd jetson-inference/tools
$ ./download-models.sh
```

## Installation de PyTorch

Le menu suivant s'affichera, appuyez sur la touche `espace` pour séléctionner la version de PyTorch. Appuyez sur `entrer` pour continuer.

<img src="https://raw.githubusercontent.com/dusty-nv/jetson-inference/python/docs/images/pytorch-installer.jpg" width="650">

Si vous voulez relancer l'**outil d'installation de Pytorch** plus tard, utilisez la commande suivante :

``` bash
$ cd jetson-inference/build
$ ./install-pytorch.sh
```

## Compilation du projet

Assurez-vous que vous êtes toujours dans le `jetson-inference/buildrépertoire`  créé ci-dessus à l'étape 3.

Après lancez la commande suivante `make` suivie de `sudo make install` pour construire les librairie, Python extension bindings, and code samples:

``` bash
$ make
$ sudo make install
$ sudo ldconfig
```
