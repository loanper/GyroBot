#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include <Servo.h>

// Déclaration des constantes
const float mySetPoint = 0.1; // Angle de référence pour la position verticale
const int servoD_Pin = 11;     // Broche pour le servo moteur droit
const int servoG_Pin = 10;     // Broche pour le servo moteur gauche

// Déclaration des variables
float angle = 0;               // Angle mesuré par le MPU6050
float erreur = 0;              // Erreur d'angle

// Coefficients PID
float Kp = 30;  // Coefficient proportionnel
float Ki = 0; // Coefficient intégral
float Kd = 0;  // Coefficient dérivé

// Variables PID
float erreurPrecedente = 0;
float sommeErreurs = 0;

// Initialisation des objets
Adafruit_MPU6050 mpu6050;
Servo servoMoteurD;
Servo servoMoteurG;

void setup() {
  // Initialisation de la communication série
  Serial.begin(115200);
  Serial.println("Initialisation terminee.");

  // Initialisation des servomoteurs
  servoMoteurD.attach(servoD_Pin);
  servoMoteurG.attach(servoG_Pin);

  // Initialisation du MPU6050
  if (!mpu6050.begin()) {
    Serial.println("Échec de la détection du MPU6050");
    while (1);
  }

  // Configuration du MPU6050
  mpu6050.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu6050.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void loop() {
  
}
