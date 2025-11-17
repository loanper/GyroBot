//Les codes utilisés pour l'initialisation de la centrale inertielle et pour son utilisation avec la bibliothèque Wire.h ont été trouvés dans les exemples de cette dernière.

#include <Wire.h>
#include <Servo.h>


//Variables pour le calcul du PID T(tangage)
float consigneVerticale = 0, consigneServo = 90;
float erreurT = 0, erreurT_prec = 0, somme_erreurT = 0; 
float kpT = 3;
float kiT = 0.1; 
float kdT = 0.07; 
float termePT = 0, termeIT = 0, termeDT = 0;
float sortiePIDT = 0;

//Variables pour le calcul du PID L(lacet)
float erreurL = 0, erreurL_prec = 0, somme_erreurL = 0; 
float kpL = 1;
float kiL = 0.5; 
float kdL = 0.01; 
float termePL = 0, termeIL =0, termeDL = 0;
float sortiePIDL = 0;

// Variables pour le calcul du PID de vitesse
float consigneVitesse = -3; // Consigne de vitesse avant/arrière
float erreurV = 0, erreurV_prec = 0, somme_erreurV = 0;
float kpV = 1.0, kiV = 0.05, kdV = 0.01;
float sortiePIDV = 0;

//Variables pour la récupération des données du MPU6050
float gyroscopeX, gyroscopeY, gyroscopeZ;
float accelerationX, accelerationY, accelerationZ;
float angleGyroX, angleGyroY,angleGyroZ,angleAccX, angleAccY;
float tangage, lacet;
float erreurTGyroX, erreurTGyroY, erreurTGyroZ, erreurTAccX, erreurTAccY;
float tempsEcoule, tempsActuel, tempsPrecedent;
float timer, dt, prevTime;


//Variables pour le mpu et les servomoteurs
const int mpu6050 = 0x68; //0x68 est l'adresse par défaut du mpu60506050 mais on l'initialise au cas où
Servo servoMoteurG;  
Servo servoMoteurD;  


void setup() {
  Serial.begin(115200);

  //Initialisation du mpu60506050 avec la bibliothèque Wire.h
  Wire.begin();                      
  Wire.beginTransmission(mpu6050);       
  Wire.write(0x6B);                 
  Wire.write(0x00);               
  Wire.endTransmission(true);        
  delay(20);

  //On définit les pins auquels sont reliés les servomoteurs
  servoMoteurG.attach(10);  
  servoMoteurD.attach(11);  

}

void loop() {
  //calculs des temps utilisés pour les calculs de lacet
  timer = micros();
  dt = (timer - prevTime) / 1000000.0; // Conversion en secondes
  prevTime = timer;
  
  //calculs des temps utilisés pour les calculs de tangage
  tempsPrecedent = tempsActuel;        
  tempsActuel = millis();            
  tempsEcoule = (tempsActuel - tempsPrecedent) / 1000.0; 

  
  
  //initialisation du gyroscope afin de lire 6 octets (2 pour chaque coordonnées)
  Wire.beginTransmission(mpu6050);
  Wire.write(0x43); 
  Wire.endTransmission(false);
  Wire.requestFrom(mpu6050, 6, true);


  // on lit les valeurs sur 2 octets et on les combinent avec | pour donner une valeur à 16 bits
  //131 correspond à la sensibilité de du gyroscope qui est calibré sur +-250dps qui est la configuration la plus précise offerte par le capteur
  // Et on corrige la valeur du gyroscope avec l'erreurT que l'on a calculé précédemment
  gyroscopeX = (Wire.read() << 8 | Wire.read()) / 131.0 + 3.5; 
  gyroscopeY = (Wire.read() << 8 | Wire.read()) / 131.0;
  gyroscopeZ = (Wire.read() << 8 | Wire.read()) / 131.0 + 1.65;
  
  
  //On multiplie les valeurs par le temps pour avoir une variable en degré et non en vitesse angulaire
  angleGyroX = angleGyroX + gyroscopeX * tempsEcoule; 
  angleGyroY = angleGyroY + gyroscopeY * tempsEcoule;
  angleGyroZ = angleGyroZ + gyroscopeZ * tempsEcoule;

  
  //initialisation de l'acceleromètre afin de lire 6 octets (2 pour chaque coordonnées)
  Wire.beginTransmission(mpu6050);
  Wire.write(0x3B); 
  Wire.endTransmission(false);
  Wire.requestFrom(mpu6050, 6, true);

  // on lit les valeurs sur 2 octets et on les combinent avec | pour donner une valeur à 16 bits
  //16384 correspond à la sensibilité de l'acceleromètre qui est calibré sur +- 2G qui est la configuration la plus précise offerte par le capteur
  accelerationX = (Wire.read() << 8 | Wire.read()) / 16384.0; 
  accelerationY = (Wire.read() << 8 | Wire.read()) / 16384.0;
  accelerationZ = (Wire.read() << 8 | Wire.read()) / 16384.0;
  
  angleAccX = (atan2(accelerationZ,  sqrt(accelerationX * accelerationX + accelerationY * accelerationY)) * 180 / PI) - 1.1; // -1.1 car l'erreurT de l'angle renvoyée par le mpu est d'environ 1.1


  //calcul des angles de tangage et de lacet 
  tangage = 0.95 * angleGyroX + 0.05 * angleAccX;
  lacet += gyroscopeY * dt;
  //roulis = 0.96 * angleGyroY + 0.04 * angleAccY;

  //calcul du PID pour le tangage
  erreurT = consigneVerticale - tangage;  
NEW SKETCH
NEW SKETCH



NEW SKETCH
  termePT = kpT * erreurT;
  somme_erreurT += erreurT; 
  termeIT = kiT * somme_erreurT;
  termeDT = kdT * (erreurT - erreurT_prec); 
  sortiePIDT = termePT + termeIT + termeDT;

NEW SKETCH

  //calcul du PID pour le lacet
  erreurL = 0 - lacet;  
  termePL = kpL * erreurL;
  somme_erreurL += erreurL; 
  termeIL = kiL * somme_erreurL;
  termeDL = kdL * (erreurL - erreurL_prec); 
  sortiePIDL = termePL + termeIL + termeDL;

   sortiePIDL = constrain(sortiePIDL, -10, 10);

  //calcul du PID pour la vitesse
   erreurV = consigneVitesse - tangage;  
  float termePV = kpV * erreurV;
  somme_erreurV += erreurV;
  float termeIV = kiV * somme_erreurV;
  float termeDV = kdV * (erreurV - erreurV_prec);
  sortiePIDV = termePV + termeIV + termeDV;

  
  servoMoteurG.write(94 + sortiePIDT );
  servoMoteurD.write(consigneServo - sortiePIDT );


  erreurT_prec = erreurT;
  erreurL_prec = erreurL;
  /*Serial.print(15.1);
  Serial.print(", ");
  Serial.print(-15.1);
  Serial.print(", ");*/
  Serial.println(lacet);
  delay(10);
}
