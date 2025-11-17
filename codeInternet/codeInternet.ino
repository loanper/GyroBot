#include <Wire.h>

#include <Servo.h>

const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

Servo myservo1;  // create servo object to control a servo
Servo myservo2;  // create servo object to control a
// twelve servo objects can be created on most boards

int pos1 = 0;
int pos2 = 0;
// variable to store the servo position
float sortie=0.00;
 int mappage;
 //derivation acceleration:
 float val_actuelle;
float valeur_prec=0;
float acc;
int i=0;
int moyenne_acc;
int val_acc;
//PID
float erreur = 0;
float erreur_prec = 0; // Pour stocker l'erreur précédente
float somme_erreur = 0; // Pour stocker la somme cumulée des erreurs
// float kp = 3;
// float ki = 0.1; // Valeur arbitraire à ajuster selon les besoins
// float kd = 0.07; // Valeur arbitraire à ajuster selon les besoins
float kp = 3;
float ki = 0.1; // Valeur arbitraire à ajuster selon les besoins
float kd = 0.07; // Valeur arbitraire à ajuster selon les besoins
float value = 0;
float consigneY;

float timer;
float dt;
float prevTime;

float erreur_yaw;
float kp2 = 3;
float ki2 = 0.01; // Valeur arbitraire à ajuster selon les besoins
float kd2 = 0.01; // Valeur arbitraire à ajuster selon les besoins

float somme_erreur_yaw;
float prop2,derivee2,integrale2;
float erreur_prec_yaw;
float value_yaw;

void setup() {
  Serial.begin(115200);
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  delay(20);
  myservo1.attach(10);  // attaches the servo on pin 9 to the servo object
  myservo2.attach(11);  // attaches the servo on pin 9 to the servo object

  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 + 3.5; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 +1.65; // Z-axis value

  consigneY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 1.58; // AccErrorY ~(-1.58)

 
}
void loop() {
  // === Read acceleromter data === //
  timer = micros();
  float dt = (timer - prevTime) / 1000000.0; // Conversion en secondes
  prevTime = timer;
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 1.1; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 1.58; // AccErrorY ~(-1.58)
  // === Read gyroscope data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0  + 3.5; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0  + 1.65;
  // Correct the outputs with the calculated error values
  GyroX = GyroX ; // GyroErrorX ~(-0.56)
  GyroY = GyroY ; // GyroErrorY ~(2)
  GyroZ = GyroZ ; // GyroErrorZ ~ (-0.8)
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw +=  GyroY * dt;
  // Complementary filter - combine acceleromter and gyro angle values
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX ;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
 
  // Print the values on the serial monitor
  Serial.println(roll);
   
  erreur=0-roll;  
  float prop = kp * erreur;

  // Terme intégral
  somme_erreur += erreur; // Ajoute l'erreur actuelle à la somme cumulée
  float integrale = ki * somme_erreur;

  // Terme dérivé
  float derivee = kd * (erreur - erreur_prec); // Calcul de la dérivée de l'erreur

  // Somme des trois termes
  value = prop + integrale + derivee;

  // Mise à jour de l'erreur précédente
  erreur_prec = erreur;

  erreur_yaw = 0-yaw;  
  float prop2 = kp2 * erreur;

  // Terme intégral
  somme_erreur_yaw += erreur_yaw; // Ajoute l'erreur actuelle à la somme cumulée
  float integrale2 = ki2 * somme_erreur_yaw;

  // Terme dérivé
  float derivee2 = kd2 * (erreur_yaw - erreur_prec_yaw); // Calcul de la dérivée de l'erreur

  // Somme des trois termes
  value_yaw = prop2 + integrale2 + derivee2;

  // Mise à jour de l'erreur précédente
  erreur_prec_yaw = erreur_yaw;

  myservo1.write(90+value+value_yaw);
  myservo2.write(90-value-value_yaw);
   Serial.println(GyroY);
  delay(10);
}
