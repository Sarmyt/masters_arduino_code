#include <Wire.h>

#define PWMA   6 // Left        
#define AIN2   A0 // Left, Forward         
#define AIN1   A1 // Left, Backwards       
#define PWMB   5 // Right          
#define BIN1   A2 // Right, Backwards   
#define BIN2   A3 // Right, Forwards        
#define ECHO   2
#define TRIG   3

const int MPU = 0x68; 
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
//float roll1, pitch1;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

void processIncomingByte (const byte);
void process_data (const char);
const unsigned int MAX_INPUT = 50;

void calculateImuError();

int distance;
int orientation;
long integral = 0;
unsigned int last_proportional = 0;
int target = 0; 
double timer = 0;

int distanceTest();
void robotForward(int, bool);
void robotBackward(int);
void robotTurn(int, bool);
void robotStop();

void setup() {
  Serial.begin(9600);
  Wire.begin();                    
  Wire.beginTransmission(MPU);      
  Wire.write(0x6B);                
  Wire.write(0x00);                  
  Wire.endTransmission(true);       

  calculateImuError();
  
  delay(20);

  pinMode(ECHO, INPUT);    
  pinMode(TRIG, OUTPUT);   
  
  pinMode(PWMA,OUTPUT);                     
  pinMode(AIN2,OUTPUT);      
  pinMode(AIN1,OUTPUT);
  
  pinMode(PWMB,OUTPUT);       
  pinMode(BIN1,OUTPUT);     
  pinMode(BIN2,OUTPUT); 
  
  robotStop();   
}

void loop() {
  while (Serial.available() > 0) {
    processIncomingByte(Serial.read());
  }
}

void process_data (char * data) {
  const char * splitData;
  splitData = strtok(data, ",");
  
  switch (splitData[0]) {
  case 'P': // Pre-intialization
    splitData = strtok(NULL, ",");
    switch (splitData[0]) {
    case 'F':
      robotForward(1000, false);
      break;

    case 'B':
      robotBackward(1000);
      break;

    case 'S':
      robotStop();
      break;

    default:
      robotTurn(atoi(splitData), false);
      break;
    }
    break;

  case 'I': // Initialization
    splitData = strtok(NULL, ",");
    orientation = atoi(splitData);
    break;

  case 'E': // Edge
    splitData = strtok(NULL, ",");
    switch (splitData[0]) {
    case 'U':
      robotTurn(180, true);
      break;

    case 'D':
      robotTurn(180, true);
      break;

    case 'L':
      robotTurn(180, true);
      break;

    case 'R':
      robotTurn(180, true);
      break;

    default:
      break;
    }    
    break;

  case 'A': // Arrival
    robotStop();
    splitData = strtok(NULL, ",");
    orientation = atoi(splitData);
    break;

  case 'R': // Run
    splitData = strtok(NULL, ",");
    robotTurn(atoi(splitData), true);
    break;

  default:
    break;
  }
}  
  
void processIncomingByte (const byte inByte) {
  static char input_line[MAX_INPUT];
  static unsigned int input_pos = 0;

  switch (inByte) {      
    case '>':
      input_line [input_pos] = 0;  
      
      process_data (input_line);
      
      input_pos = 0;  
      break;

    default:
      if (input_pos < (MAX_INPUT - 1))
        input_line [input_pos++] = inByte;
      break;
  }  
} 

void getOrientation() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); 
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; 
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; 
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; 
 
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX; 
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorY; 

  previousTime = currentTime;        
  currentTime = millis();            
  elapsedTime = (currentTime - previousTime) / 1000; 
  Wire.beginTransmission(MPU);
  Wire.write(0x43); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; 
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;

  GyroX = GyroX - GyroErrorX; 
  GyroY = GyroY - GyroErrorY; 
  GyroZ = GyroZ - GyroErrorZ; 

  gyroAngleX = gyroAngleX + GyroX * elapsedTime; 
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  
  yaw = yaw + GyroZ * elapsedTime;
  //roll1 = 0.96 * gyroAngleX + 0.04 * accAngleX;
  //pitch1 = 0.96 * gyroAngleY + 0.04 * accAngleY;  

  roll = gyroAngleX;
  pitch = gyroAngleY;
}

void calculateImuError() {
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;

    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }

  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;

  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();

    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }

  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
}

int distanceTest()         
{
  digitalWrite(TRIG, LOW);   
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);  
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);    
  float Fdistance = pulseIn(ECHO, HIGH);  
  Fdistance= Fdistance/58;             
  return (int)Fdistance;
}  

void robotForward(int duration, bool isRun)
{
  getOrientation();
  target = roll;

  unsigned long destination = millis() + duration;

  analogWrite(PWMA, 0); // Left
  analogWrite(PWMB, 0); // Right
 
  digitalWrite(AIN1,LOW);
  digitalWrite(AIN2,HIGH);
  digitalWrite(BIN1,LOW);  
  digitalWrite(BIN2,HIGH); 

  while (millis() < destination) {
  getOrientation();

  int proportional = roll - target;
  int derivative = proportional - last_proportional;
  integral += proportional;
  last_proportional = proportional;

  int power_difference = proportional/20 + integral/10000 + derivative*10; 

  const int maximum = 60;

  if (power_difference > maximum)
    power_difference = maximum;
  if (power_difference < - maximum)
    power_difference = - maximum;

  if (power_difference < 0)
  {
     analogWrite(PWMA,maximum + power_difference);
     analogWrite(PWMB,maximum);
   }
   else
   {
      analogWrite(PWMA,maximum);
      analogWrite(PWMB,maximum - power_difference);
   }      
   
   distance = distanceTest();

   if (distance <= 3) {
     robotStop();
   }

   if (isRun) {
       if (Serial.available() > 0) {
           destination = millis() - 1;
       } 
   }
   
  }
  integral = 0;
  last_proportional = 0;
  robotStop();
}

void robotBackward(int duration)
{
  getOrientation();
  target = roll;

  unsigned long destination = millis() + duration;

  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0); 
 
  digitalWrite(AIN1,HIGH);
  digitalWrite(AIN2,LOW);
  digitalWrite(BIN1,HIGH);  
  digitalWrite(BIN2,LOW); 

  while (millis() < destination) {
  getOrientation();

  int proportional = roll - target;
  int derivative = proportional - last_proportional;
  integral += proportional;
  last_proportional = proportional;

  int power_difference = proportional/20 + integral/10000 + derivative*10; 

  const int maximum = 60;

  if (power_difference > maximum)
    power_difference = maximum;
  if (power_difference < - maximum)
    power_difference = - maximum;

  if (power_difference < 0)
  {
     analogWrite(PWMB,maximum + power_difference);
     analogWrite(PWMA,maximum);
   }
   else
   {
      analogWrite(PWMB,maximum);
      analogWrite(PWMA,maximum - power_difference);
   }      
  }
  integral = 0;
  last_proportional = 0;
  robotStop();
}

void robotTurn(int angle, bool isRun)
{
  unsigned long timer = millis();
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0); 
  
  if (angle > 180) {
    angle = angle - 360; 
  }
  
  getOrientation();
  target = roll + angle;  
  //Serial.println(roll);
  
  while (abs(target - roll) > 2) {
  if ((millis() - timer) > 8000) {
    break;
  }
    
  getOrientation();
    
  int proportional = roll - target;
  int derivative = proportional - last_proportional;
  integral += proportional;
  last_proportional = proportional;

  int power_difference = proportional/40 + integral/500 + derivative*10; 
  power_difference = abs(power_difference);
  const int maximum = 60;

  if (power_difference > maximum)
    power_difference = maximum;
  if (power_difference < 0)
    power_difference = 0;

  if (target < roll) {
      digitalWrite(AIN1,LOW);
      digitalWrite(AIN2,HIGH);
      digitalWrite(BIN1,HIGH);  
      digitalWrite(BIN2,LOW); 
   }
   else {
       digitalWrite(AIN1,HIGH);
       digitalWrite(AIN2,LOW);
       digitalWrite(BIN1,LOW);  
       digitalWrite(BIN2,HIGH); 
   }      
   
   analogWrite(PWMB, power_difference);
   analogWrite(PWMA, power_difference);
  }
  integral = 0;
  last_proportional = 0;
  robotStop();

  if (isRun) {
      robotForward(1000, isRun);
  } 
}

void robotStop()
{
  analogWrite(PWMA,0);
  analogWrite(PWMB,0);
  digitalWrite(AIN1,LOW);
  digitalWrite(AIN2,LOW);
  digitalWrite(BIN1,LOW); 
  digitalWrite(BIN2,LOW);  
}
