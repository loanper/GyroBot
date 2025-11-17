//Les codes utilisés pour l'initialisation de la centrale inertielle et pour son utilisation avec la bibliothèque Wire.h ont été trouvés dans les exemples de cette dernière.

#include <Wire.h>
#include <Servo.h>


//Variables pour le calcul du PID T(tangage)
float consigneVerticale = 0, consigneServo = 90;
float erreurT = 0, erreurT_prec = 0, somme_erreurT = 0; 
float kpT = 3;
float kiT = 0.1; 
float kdT = 0.07; 
float termePT, termeIT, termeDT;
float sortiePIDT = 0;

//Variables pour le calcul du PID L(lacet)
float erreurL = 0, erreurL_prec = 0, somme_erreurL = 0; 
float kpL = 4;
float kiL = 0.1; 
float kdL = 0.013; 
float termePL, termeIL, termeDL;
float sortiePIDL = 0;

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
  
  //calculs des temps utilisés pour les calculs de tangage
  tempsPrecedent = tempsActuel;        
  tempsActuel = millis();            
  tempsEcoule = (tempsActuel - tempsPrecedent) / 1000; 

  //calculs des temps utilisés pour les calculs de lacet
  timer = micros();
  float dt = (timer - prevTime) / 1000000.0; // Conversion en secondes
  prevTime = timer;
  
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
  angleAccY = (atan(-1 * accelerationX / sqrt(pow(accelerationY, 2) + pow(accelerationZ, 2))) * 180 / PI) + 1.58; // erreurTAccY ~(-1.58)



  //calcul des angles de tangage et de lacet 
  tangage = 0.95 * angleGyroX + 0.05 * angleAccX;
  lacet += gyroscopeY * dt;
  //roulis = 0.96 * angleGyroY + 0.04 * angleAccY;

  //calcul du PID pour le tangage
  erreurT = consigneVerticale - tangage;  
  termePT = kpT * erreurT;
  somme_erreurT += erreurT; 
  termeIT = kiT * somme_erreurT;
  termeDT = kdT * (erreurT - erreurT_prec); 
  sortiePIDT = termePT + termeIT + termeDT;

  //calcul du PID pour le lacet
  erreurL = consigneVerticale - lacet;  
  termePL = kpL * erreurL;
  somme_erreurL += erreurL; 
  termeIL = kiL * somme_erreurL;
  termeDL = kdL * (erreurL - erreurL_prec); 
  sortiePIDL = termePL + termeIL + termeDL;

  
  servoMoteurG.write(consigneServo + sortiePIDT);
  servoMoteurD.write(consigneServo - sortiePIDT);


  erreurT_prec = erreurT;
  erreurL_prec = erreurL;
  Serial.print(15.1);
  Serial.print(", ");
  Serial.print(-15.1);
  Serial.print(", ");
  Serial.println(lacet);
  delay(10);
}
