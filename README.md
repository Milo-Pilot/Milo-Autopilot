# Source file of the Milo autopilot.


website:   www.milo-autopilot.com


La version française suit.


## Software and hardware

Runs on a arduino-compatible nano board.
It may be edited using the arduino IDE (the arduino programming platform).
It is connected to a Bosch-sensortec BMX055 IMU (Inertial Measurement Unit, 9 axes, Magnetometer + Gyrometer + Accelerometer)
It is also connected to a h-bridge for controling the DC motor of the autopilot.

It uses the FaBo9Axis_BMX055 library to communicate with the BMX055 IMU. This libray has been slightly modified in order to
maximize the accuracy and stability of the IMU, and renamed to "FaBo9Axis_BMX055_Milo"


## User interface

### On/off switch

This program is run at power-up (when this switch is set to "on")

### Pilot engagement

it automatically sets the "engaging switch" to "on".

### Calibration switch

- set to "on" : pilot set to calibration mode.
- set to "off" : pilot set to normal operation mode.


## What this program do : 

### If the calibration switch is "on":

  User is requested to execute 2 circles with the boat. While doing this, the program computes data that will be used later to calculate the calibration     offsets. When this is done (at the time user sets the calibration switch to "off"), these calibration offsets get computed from these data and stored in   the EEPROM of the board, for further use.

### If the calibration switch is "off" :
  
  The program constantly computes the current heading angle and current angular speed of the boat.
  When user engages the pilot, a sensor detects that pilot is engaged. This sensor automatically switches the "engaging switch" on.
  At the time the pilot is engaged, the heading angle becomes the "target heading angle" 
  (Deviation heading angle) is :  (current heading angle) - (target heading angle)

  the program uses deviation heading and angular speed to compute whether the motor of the pilot must be run or not. It uses a PID (without "I"). 
  Motor must be run if the command computed by PID is outside the deadband. 
  If motor must be run, h-bridge connected to the board is activated, and current delivered to the motor is measured.
  If this current is too high (for example if the tiller is blocked by an obstacle), the signal to the h-bridge is paused.



------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------



# Fichier source du pilote automatique Milo.

(french version)


## Logiciel et matériel

Executable sur une carte nano compatible arduino.
Il peut être modifié à l'aide de l'IDE arduino (la plate-forme de programmation arduino).
Il est connecté à une IMU Bosch-sensortec BMX055 (Inertial Measurement Unit, 9 axes, Magnetomètre + Gyromètre + Acceleromètre)
Il est également connecté à un pont en H pour contrôler le moteur à courant continu du pilote automatique.

Il utilise la bibliothèque FaBo9Axis_BMX055 pour communiquer avec l'IMU BMX055. Cette bibliothèque a été légèrement modifiée afin de
maximiser la précision et la stabilité de l'IMU, et renommé en "FaBo9Axis_BMX055_Milo"


## Interface utilisateur

### Interrupteur marche / arrêt

Ce programme est exécuté à la mise sous tension (lorsque cet interrupteur est positionné sur "on")

### Embrayage du pilote

Lorsque l’utilisateur embraye le pilote, cela positionne automatiquement l’"interrupteur d’embrayage" sur "on".

### Interrupteur de calibration

- mis sur "on" : pilote est mis en mode calibration.
- mis sur "off" : pilote mis en mode de fonctionnement normal.


## Que fait ce programme :

### Si l’interrupteur de calibration est "on":

L'utilisateur doit exécuter 2 cercles avec le bateau; ce faisant, le programme calcule des données qui seront utilisées pour déterminer les offsets de calibration. Lorsque les 2 cercles sont éxécutés, l'uitilisateur place l'interrupteur de calibration sur "off" et à cet instant , les offsets de calibration sont calculés et stockés dans l'EEPROM de la carte, pour une utilisation ultérieure.


### Si l’interrupteur de calibration est "off" :
  
Le programme calcule en permanence le cap courant et la vitesse angulaire du bateau.
Lorsque l'utilisateur embraye le pilote, un capteur détecte que le pilote est embrayé. Ce capteur commute automatiquement l’interrupteur d’embrayage.

A l’instant où le pilote est embrayé, le cap devient "le cap cible"
(déviation de cap) égale : (cap courant) - (cap cible)

le programme utilise la déviation de cap et la vitesse angulaire pour calculer si le moteur du pilote doit tourner ou non. Il utilise un PID (sans "I").
Le moteur doit tourner si la commande calculée par PID est en dehors de la bande morte. 
Si le moteur doit tourner, le pont en h connecté à la carte est activé et le courant fourni au moteur est mesuré.
Si ce courant est trop élevé (par exemple si la barre est bloquée par un obstacle), l’activation du pont en h est mise en pause.
