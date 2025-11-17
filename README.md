# Gyrobot — Projet Arduino

Développement d'un robot auto-équilibrant à deux roues utilisant le capteur MPU6050 pour mesurer l'inclinaison et la vitesse. Le robot se stabilise verticalement, maintient une direction et conserve une vitesse constante. Les défis incluent l'intégration des composants électroniques, la programmation des contrôleurs PID et l'adaptation aux variations des composants.

## Description

Gyrobot est un projet basé sur des cartes compatibles Arduino. Il regroupe des sketches expérimentaux et utilitaires : calcul d'angles, code de communication (internet), tests et exemples pour soutenances.

## Arborescence principale

- `script.ino` — sketch principal placé à la racine (point d'entrée testé).
- `calculAngle/` — code pour le calcul d'angles (`calculAngle.ino`).
- `codeInternet/` — sketch lié à la communication/internet (`codeInternet.ino`).
  - `Nouveau dossier/` — fichiers additionnels (si présents).
- `CodesSoutenances/CodeET2/` — sketch(s) pour la soutenance (`CodeET2.ino`).
- `testblk/` — exemples/tests (`testblk.ino`).

> Remarque : chaque sous-dossier contient un fichier `.ino` autonome. Ouvrez le fichier `.ino` correspondant pour travailler dessus.

## Prérequis

- Arduino IDE (recommandé) ou PlatformIO (VS Code) installé.
- Pilotes USB pour votre carte Arduino (ex. CH340, FTDI, ou pilotes officiels Arduino selon la carte).
- Câble USB adapté à votre carte.

## Compilation et téléversement (Arduino IDE)

1. Ouvrez l'IDE Arduino.
2. Dans `Fichier > Ouvrir`, naviguez et ouvrez le fichier `.ino` que vous voulez utiliser (ex. `script.ino` ou `calculAngle/calculAngle.ino`).
3. Sélectionnez le type de carte dans `Outils > Type de carte` (ex. Arduino Uno, Nano, etc.).
4. Sélectionnez le port série dans `Outils > Port`.
5. Cliquez sur le bouton `Vérifier` (compiler) puis sur `Téléverser` pour envoyer le sketch sur la carte.

Conseil : si le code utilise la communication série, ouvrez le Moniteur Série (`Outils > Moniteur Série`) et vérifiez la vitesse (baud) indiquée dans le sketch (`Serial.begin(...)`).

## Compilation et téléversement (PlatformIO / VS Code)

1. Ouvrez le dossier racine dans VS Code.
2. Installez l'extension PlatformIO si ce n'est pas déjà fait.
3. Créez un projet PlatformIO pointant vers le dossier contenant le `.ino`, ou convertissez le sketch en projet PlatformIO.
4. Utilisez les commandes PlatformIO (build / upload) pour compiler et téléverser.

## Tester et debug

- Utilisez le Moniteur Série pour lire les logs et vérifier le comportement.
- Ajoutez des appels `Serial.println()` dans le sketch pour déboguer.
- Pour des tests matériels, vérifiez le câblage et l'alimentation avant le téléversement.

