
#include <Wire.h>

// Adresse du MPU6050
const int MPU6050_ADDRESS = 0x68;

void setup() {
  Serial.begin(9600);

  // Initialisation du MPU6050
  Wire.begin();
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x6B); // Régistre de configuration du MPU6050
  Wire.write(0);    // Valeur pour activer le MPU6050
  Wire.endTransmission(true);
  delay(20); // Laisser le MPU6050 s'initialiser
}

void loop() {
  // Lecture des données du gyroscope
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x43); // Régistre de départ pour les données du gyroscope
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDRESS, 6, true);

  // Lecture des valeurs brutes du gyroscope (16 bits par axe)
  int16_t rawGyroX = Wire.read() << 8 | Wire.read();
  int16_t rawGyroY = Wire.read() << 8 | Wire.read();
  int16_t rawGyroZ = Wire.read() << 8 | Wire.read();

  // Conversion des valeurs brutes en degrés par seconde (dps)
  float gyroX = (rawGyroX / 131.0)+ 4; // Sensibilité du gyroscope pour ±250 dps
  float gyroY = rawGyroY / 131.0;
  float gyroZ = rawGyroZ / 131.0 + 1.6;

  // Affichage des valeurs du gyroscope
  Serial.print("Gyro X:   ");
  Serial.print(gyroX);
  Serial.print("   Gyro Y:   ");
  Serial.print(gyroY);
  Serial.print("   Gyro Z:   ");
  Serial.println(gyroZ);

  delay(100); // Pause entre les lectures
}


/*
#include <Wire.h>

// Constantes pour l'adresse du MPU6050
const int MPU6050_ADDRESS = 0x68;

// Variables pour les angles roll et pitch
float roll = 0;
float pitch = 0;

void setup() {
  Serial.begin(9600);

  // Initialisation du MPU6050
  Wire.begin();
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x6B); // Régistre de configuration du MPU6050
  Wire.write(0);    // Valeur pour activer le MPU6050
  Wire.endTransmission(true);
  delay(20); // Laisser le MPU6050 s'initialiser
}

void loop() {
  // Lecture des données de l'accéléromètre
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x3B); // Régistre de départ pour les données de l'accéléromètre
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDRESS, 6, true);

  // Lecture des valeurs brutes de l'accéléromètre
  int16_t rawAccX = Wire.read() << 8 | Wire.read();
  int16_t rawAccY = Wire.read() << 8 | Wire.read();
  int16_t rawAccZ = Wire.read() << 8 | Wire.read();

  // Conversion des valeurs brutes en g (accélération)
  float accX = rawAccX / 16384.0; // Sensibilité de l'accéléromètre ±2g
  float accY = rawAccY / 16384.0;
  float accZ = rawAccZ / 16384.0;

  // Calcul des angles roll et pitch à partir de l'accéléromètre
  roll = (atan2(accZ, sqrt(accX*accX+accY*accY)) * 180 / PI) - 1;
  float  yaw = atan2(accX, accZ) * 180 / PI;
  pitch = (atan(-1 * accY / sqrt(pow(accZ, 2) + pow(accX, 2))) * 180 / PI);


  // Affichage des angles roll et pitch
  Serial.print("AccX: ");
  Serial.print(accX -0.05);
  Serial.print("AccY: ");
  Serial.print(accY - 1);
  Serial.print("AccZ: ");
  Serial.println(accZ- 0.05);
  



  delay(100); // Pause entre les lectures
}*/
