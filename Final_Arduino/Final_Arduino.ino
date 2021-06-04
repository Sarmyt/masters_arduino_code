#include <Wire.h>

#define PWMA   6 // Left wheel power pin        
#define AIN2   A0 // Left wheel forward power pin         
#define AIN1   A1 // Left wheel backwards power pin       
#define PWMB   5 // Right wheel power pin          
#define BIN1   A2 // Right wheel backwards power pin   
#define BIN2   A3 // Right wheel forwards power pin  
#define ECHO   2 // Echo pin for ultrasonic sensor
#define TRIG   3 // Trig pin for ultrasonic sensor

const int MPU = 0x68; // MPU6050 register
float AccX, AccY, AccZ; // Variables for storing accelerometer information
float GyroX, GyroY, GyroZ; // Variables for storing gyroscope information
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ; // Variables for storing accelerometer and gyroscopic angles
float roll, pitch, yaw; // Variables for storing roll, pitch and yaw information
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ; // Variables for storing accelerometer and gyroscopic reading error
float elapsedTime, currentTime, previousTime; // Variables for storing time measurements
int c = 0;

void processIncomingByte (const byte); // Function to process incoming byte data
void process_data (const char); // Function to process collection of bytes
const unsigned int MAX_INPUT = 50; // Max message character length

void calculateImuError(); // Function to calculate accelerometer and gyroscopic reading errors

int distance; // Storing distance reading from ultrasonic sensor
int orientation; // Storing orientation of robot
long integral = 0; // Variable used for PID calculation (integral term)
unsigned int last_proportional = 0; // Variable used for storing previous proportional error
int target = 0; // Variable for storing target orientation of robot
double timer = 0; // Timer variable for timing behaviour of robot

int distanceTest(); // Function for testing distance from robot
void robotForward(int, bool); // Function to make robot move forward
void robotBackward(int); // Function to make robot move backwards
void robotTurn(int, bool); // Function to make robot turn
void robotStop(); // Function to make robot stop

void setup() {
  Serial.begin(9600);
  Wire.begin();                    
  Wire.beginTransmission(MPU);      // Begin communication with MPU6050 
  Wire.write(0x6B);                
  Wire.write(0x00);                  
  Wire.endTransmission(true);       

  calculateImuError(); // Get accelerometer and gyroscopic erros
  
  delay(20);

  // Define all pins

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
  while (Serial.available() > 0) { // Look for availability of serial data
    processIncomingByte(Serial.read()); // If serial data is available then process it
  }
}

void process_data (char * data) { // Decode message that was received by robot
  const char * splitData;
  splitData = strtok(data, ","); // Split message by "," token
  
  switch (splitData[0]) {
  case 'P': // Pre-intialization message type
    splitData = strtok(NULL, ",");
    switch (splitData[0]) {
    case 'F':
      robotForward(1000, false); // Move robot forward for one second
      break;

    case 'B':
      robotBackward(1000); // Move robot backwards for one second
      break;

    case 'S':
      robotStop(); // Stop robot
      break;

    default:
      robotTurn(atoi(splitData), false); // Turn robot by a given angle
      break;
    }
    break;

  case 'I': // Initialization message type
    splitData = strtok(NULL, ",");
    orientation = atoi(splitData);
    break;

  case 'E': // Edge message type
    splitData = strtok(NULL, ",");
    switch (splitData[0]) {
    case 'U':
      robotTurn(180, true); // Robot has hit an edge, turn robot by 180 degrees
      break;

    case 'D':
      robotTurn(180, true); // Robot has hit an edge, turn robot by 180 degrees
      break;

    case 'L':
      robotTurn(180, true); // Robot has hit an edge, turn robot by 180 degrees
      break;

    case 'R':
      robotTurn(180, true); // Robot has hit an edge, turn robot by 180 degrees
      break;

    default:
      break;
    }    
    break;

  case 'A': // Arrival message type
    robotStop(); // Robot has arrived where it needs to be, stop it
    splitData = strtok(NULL, ",");
    orientation = atoi(splitData);
    break;

  case 'R': // Run message type
    splitData = strtok(NULL, ",");
    robotTurn(atoi(splitData), true); // Turn robot by desired amount indicated in message
    break;

  default:
    break;
  }
}  
  
void processIncomingByte (const byte inByte) {
  static char input_line[MAX_INPUT];
  static unsigned int input_pos = 0;

  switch (inByte) {      
    case '>': // This character indicates end of message
      input_line [input_pos] = 0;  
      
      process_data (input_line);
      
      input_pos = 0;  
      break;

    default: // Message has not ended so move onto next byte
      if (input_pos < (MAX_INPUT - 1))
        input_line [input_pos++] = inByte;
      break;
  }  
} 

void getOrientation() { // This code is derived from Dejan at https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/

  previousTime = currentTime;        
  currentTime = millis();            
  elapsedTime = (currentTime - previousTime) / 1000; // Time elasped from previous reading
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Access first gyroscope register
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Access data from all 6 gyroscopic registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // Divide data collected by sensitivity scale factor
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0; // Divide data collected by sensitivity scale factor
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0; // Divide data collected by sensitivity scale factor

  GyroX = GyroX - GyroErrorX; // Adjust for gyroscopic error in x-axis
  GyroY = GyroY - GyroErrorY; // Adjust for gyroscopic error in y-axis
  GyroZ = GyroZ - GyroErrorZ; // Adjust for gyroscopic error in z-axis

  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // Integrate gyroscope acceleration over time to get change in angle
  gyroAngleY = gyroAngleY + GyroY * elapsedTime; // Integrate gyroscope acceleration over time to get change in angle
  gyroAngleZ = gyroAngleZ + GyroZ * elapsedTime; // Integrate gyroscope acceleration over time to get change in angle
  
  yaw = gyroAngleZ; 
  roll = gyroAngleX;
  pitch = gyroAngleY;
}

void calculateImuError() { // This code is derived from Dejan at https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/

  while (c < 200) { // Collect gyroscope data 200 times
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

  GyroErrorX = GyroErrorX / 200; // Average out the 200 readings to obtain average error reading
  GyroErrorY = GyroErrorY / 200; // Average out the 200 readings to obtain average error reading
  GyroErrorZ = GyroErrorZ / 200; // Average out the 200 readings to obtain average error reading
}

int distanceTest()         
{
  digitalWrite(TRIG, LOW); // Set trig pin off
  delayMicroseconds(2); // Wait two milliseconds
  digitalWrite(TRIG, HIGH); // Set trig pin on
  delayMicroseconds(10); // Wait 10 milliseconds
  digitalWrite(TRIG, LOW); // Set trig pin off    
  float Fdistance = pulseIn(ECHO, HIGH); // Turn on echo pin to listen for reflection of emitted signal
  Fdistance= Fdistance/59; // Calculate distance to obstacle             
  return (int)Fdistance;
}  

void robotForward(int duration, bool isRun) // Moves robot forward for a given duration
{
  getOrientation(); // Get orientation of robot
  target = roll; // Set heading of robot, this must be maintained constant for straightline motion

  unsigned long destination = millis() + duration; // Determine length of time to move forward

  analogWrite(PWMA, 0); // Left wheel power to zero
  analogWrite(PWMB, 0); // Right wheel power to zero
 
  digitalWrite(AIN1,LOW);
  digitalWrite(AIN2,HIGH); // Left wheel to move forward
  digitalWrite(BIN1,LOW);  
  digitalWrite(BIN2,HIGH); // Right wheel to move foward

  while (millis() < destination) {
  getOrientation();

  int proportional = roll - target; // Calculate proportional error for PID
  int derivative = proportional - last_proportional; // Calculate derivative error for PID
  integral += proportional; // Calculate integral error for PID
  last_proportional = proportional; // Store proportional error for next time

  int power_difference = proportional/20 + derivative*10; // Use PD controller to determine appropriate control action

  const int maximum = 60;

  if (power_difference > maximum)
    power_difference = maximum;
  if (power_difference < - maximum)
    power_difference = - maximum;

  if (power_difference < 0)
  {
     analogWrite(PWMA,maximum + power_difference); // Robot orientation is moving right, spin right wheel faster than left wheel
     analogWrite(PWMB,maximum);
   }
   else
   {
      analogWrite(PWMA,maximum);
      analogWrite(PWMB,maximum - power_difference); // Robot orientation is moving left, spin left wheel faster than right wheel
   }      
   
   distance = distanceTest();

   if (distance <= 3) {
     robotStop(); // If obstacle in the way then stop moving
   }

   if (isRun) { // If robot receives another message then stop the forward motion
       if (Serial.available() > 0) {
           destination = millis() - 1;
       } 
   }
   
  }
  integral = 0; // Reset variable to 0
  last_proportional = 0; // Reset variable to 0
  robotStop(); // Stop the robot
}

void robotBackward(int duration) // Look at "robotForwards" for reference, this function just makes the robot move backwards
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

  int power_difference = proportional/20 + derivative*10; 

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

void robotTurn(int angle, bool isRun) // Spins robot by a given angle amount
{
  unsigned long timer = millis();
  analogWrite(PWMA, 0); // Left wheel power to zero
  analogWrite(PWMB, 0); // Right wheel power to zero
  
  if (angle > 180) { // Normalize angle between -180 to 180 degrees
    angle = angle - 360; 
  }
  
  getOrientation(); // Find current orientation of robot
  target = roll + angle; // Determine desired orientation of robot
  //Serial.println(roll);
  
  while (abs(target - roll) > 2) { // Until the robot is within 2 degrees of desired angle, keep turning robot
  if ((millis() - timer) > 8000) { // If robot has been turning for over 8 seconds, then stop turning
    break;
  }
    
  getOrientation();
    
  int proportional = roll - target; // Calculate proportional error for PID
  int derivative = proportional - last_proportional; // Calculate derivative error for PID
  integral += proportional; // Calculate integral error for PID
  last_proportional = proportional; // Store proportional error for next time

  int power_difference = proportional/40 + integral/500 + derivative*10; // Use PID controller to determine appropriate control action
  power_difference = abs(power_difference);
  const int maximum = 60;

  if (power_difference > maximum)
    power_difference = maximum;
  if (power_difference < 0)
    power_difference = 0;

  if (target < roll) { // If clockwise rotation is required
      digitalWrite(AIN1,LOW);
      digitalWrite(AIN2,HIGH);
      digitalWrite(BIN1,HIGH);  
      digitalWrite(BIN2,LOW); 
   }
   else { // If counter-clockwise rotation is required
       digitalWrite(AIN1,HIGH);
       digitalWrite(AIN2,LOW);
       digitalWrite(BIN1,LOW);  
       digitalWrite(BIN2,HIGH); 
   }      
   
   analogWrite(PWMB, power_difference); // Use PID controller output to determine appropriate control action
   analogWrite(PWMA, power_difference); // Use PID controller output to determine appropriate control action
  }
  integral = 0; // Reset variable to 0
  last_proportional = 0; // Reset variable to 0
  robotStop(); // Stop the robot

  if (isRun) {
      robotForward(1000, isRun); // When done turning, move robot forwards for one second
  } 
}

void robotStop()
{ // Stop the robot by stopping all motors
  analogWrite(PWMA,0);
  analogWrite(PWMB,0);
  digitalWrite(AIN1,LOW);
  digitalWrite(AIN2,LOW);
  digitalWrite(BIN1,LOW); 
  digitalWrite(BIN2,LOW);  
}
