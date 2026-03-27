# 🤖 ArUco Robot — Vision-Based Navigation & Control

Ce projet permet de piloter un robot **Adeept PiCar-Pro** en utilisant des marqueurs **ArUco** pour la navigation autonome et le contrôle précis du bras robotisé.

Le système est divisé en deux parties : une station de contrôle (PC/Mac) qui traite le flux vidéo et un serveur (Raspberry Pi) qui exécute les commandes motrices.

---

## 🏗️ Architecture du Projet

### 1. Contrôleur Visuel (`control_visual.py`) — *Côté PC*

C'est le cerveau du système. Il utilise OpenCV pour :

- Détecter les marqueurs ArUco.
- Calculer la distance et l'angle entre le robot (ID 1) et sa cible (ID 2 ou 3).
- Afficher un **HUD (Heads-Up Display)** industriel avec télémétrie en temps réel.
- Envoyer des commandes de mouvement (FORWARD, LEFT, RIGHT, STOP) via TCP.

### 2. Serveur Robot (`server.py`) — *Côté Raspberry Pi*

Un script léger qui tourne sur le robot et :

- Écoute les commandes TCP sur le port `9999`.
- Pilote le driver PWM **PCA9685**.
- Gère le servo de direction et les moteurs DC arrière.

### 3. Séquence de Manipulation (`pick_exemple.py`)

Un exemple de contrôle du bras 4-DOF (Degrés de Liberté) :

- Séquences fluides de "Pick" (saisie) et "Rest" (repos).
- Gestion de la base, de l'épaule, du coude et de la pince.

### 4. Outil de Vérification (`aruco_verify.py`)

Outil de diagnostic pour tester la détection sans faire bouger le robot. Permet de vérifier les IDs des marqueurs et la calibration de la caméra.

---

## 🛠️ Configuration & Installation

### Prérequis

- **PC** : Python 3.10+, `opencv-contrib-python`, `numpy`.
- **Robot (Pi)** : Bibliothèques `adafruit-circuitpython-pca9685`, `adafruit-circuitpython-motor`.

### Installation

1. Clonez ce dépôt sur votre PC et sur le Raspberry Pi.
2. Configurez l'adresse IP de votre robot dans `control_visual.py` :

   ```python
   ROBOT_IP = "192.168.137.254"  # À adapter à votre réseau
   ```

---

## 🚀 Utilisation

### Étape 1 : Lancer le serveur sur le Robot

```bash
python3 server.py
```

### Étape 2 : Lancer le contrôleur sur le PC

```bash
python3 control_visual.py
```

### Contrôles (Interface PC)

- **[ESPACE]** : Démarrer / Pause la navigation.
- **[N]** : Basculer vers la Destination 2 (ArUco #3).
- **[R]** : Réinitialiser (Stop robot + retour vers Destination 1).
- **[Q]** : Quitter proprement.

---

## 🏷️ Mapping des Marqueurs ArUco

Le projet utilise le dictionnaire `DICT_4X4_50`.

- **ID 0** : **BASE** — Repère au sol pour l'échelle spatiale.
- **ID 1** : **ROBOT** — Fixé sur le dessus du robot.
- **ID 2** : **DEST 1** — Première cible de navigation.
- **ID 3** : **DEST 2** — Deuxième cible (activée avec la touche `N`).

---

*Projet réalisé dans le cadre du module Mechatronic - ESILV A4 S2.*
