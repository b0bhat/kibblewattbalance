#include <SPI.h>
#include <Wire.h>

const byte CS_pin = 10;
uint16_t command = 0b1111111111111111;

float coil1_r = 1700.0;
float res1 = 9850.0;

bool forceMode = false;
bool startFlag = false;

float pos;

int lowbyte; //raw angle 7:0
word highbyte; //raw angle 7:0 and 11:8
int rawAngle; //final raw angle 
float degAngle;
int magnetStatus = 0;

float calibratedAngle = 0;

float mass;
#define GRAVITY 9.81
#define ARM_LENGTH 0.2

uint8_t duty_cycle = 0;

// input
char receivedChar;
boolean newData = false;

void setup() {
  //Serial.begin(9600);
  Serial.begin(9600);
  // while (!Serial) {
  //   ;
  // }
}

void begininit() {
  // setup hall effect sensor
  Wire.begin(); //start i2C  
	Wire.setClock(800000L);
  checkMagnetPresence();
  readRawAngle();
  calibratedAngle = degAngle;

  // Serial.println("initialized");
  // Serial.print("calibrated angle: ");
  Serial.println(degAngle);
}

// set to digital pins instead
#define COILAP 9
#define COILAN 10
//#define COILAGND 11

void force_mode_setup() {
  pinMode(COILAP, OUTPUT);
  pinMode(COILAN, OUTPUT);
  analogWrite(COILAP, 0);
  analogWrite(COILAN, 0);
  delay(1000);
  forceMode = true;
}

void force_mode() {
  // assuming pos is being updated
  Serial.println(fabs(degAngle-calibratedAngle));
  if(fabs(degAngle-calibratedAngle) <= 0.1) {
    update_mass();
    Serial.print("Measured mass (g): ");
    Serial.println(mass);
    resetState();
  } else if(duty_cycle < 250){ // Position is not yet balanced
    //Serial.print("pushing: ");
    //Serial.println(duty_cycle);
    analogWrite(COILAP, duty_cycle);
    duty_cycle  = duty_cycle + 1;
    delay(50);
    readRawAngle();
    //digitalWrite(CALIBLED, LOW);
  } else {
    Serial.println("end");
  }

}

void readRawAngle() { 
  //7:0 - bits
  Wire.beginTransmission(0x36); //connect to the sensor
  Wire.write(0x0D); //figure 21 - register map: Raw angle (7:0)
  Wire.endTransmission(); //end transmission
  Wire.requestFrom(0x36, 1); //request from the sensor
  
  while(Wire.available() == 0); //wait until it becomes available 
  lowbyte = Wire.read(); //Reading the data after the request
 
  //11:8 - 4 bits
  Wire.beginTransmission(0x36);
  Wire.write(0x0C); //figure 21 - register map: Raw angle (11:8)
  Wire.endTransmission();
  Wire.requestFrom(0x36, 1);
  
  while(Wire.available() == 0);  
  highbyte = Wire.read();
  highbyte = highbyte << 8; //shifting to left

  rawAngle = highbyte | lowbyte; //int is 16 bits (as well as the word)
  degAngle = rawAngle * 0.087890625;
  //Serial.println(degAngle, 5);
}

double get_current(){
  double current = (((duty_cycle / 255.0) * 100.0) * 5.0)/83.1; // 83.1 is resistance?
  return current;
}

void update_mass(){
  double bl = get_avg_bl();
  double i = get_current();
  mass = bl * (i/GRAVITY);
}

double get_avg_bl() {
  return 3;
}

void recvOneChar() {
    if (Serial.available() > 0) {
        receivedChar = Serial.read();
        newData = true;
    }
}

void resetState() {
  startFlag = false;
  forceMode = false;
  pinMode(COILAP, OUTPUT);
  pinMode(COILAN, OUTPUT);
  analogWrite(COILAP, 0);
  analogWrite(COILAN, 0);
  degAngle = 0;
  duty_cycle = 0;
  delay(100);
  Serial.println("reset");
}

void loop() {
  recvOneChar();
  if (!startFlag) {
    if (receivedChar == 's') {
      startFlag = true;
      begininit();
    } else {
      return;
    }
  }

  if (receivedChar == 'f') {
    force_mode_setup();
  }

  if (receivedChar == 'r') {
    resetState();
  }

  if (forceMode) {
      // do forcemode
      readRawAngle();
      force_mode();
  }

  //force_mode_test();
}

void checkMagnetPresence() {  
  //This function runs in the setup() and it locks the MCU until the magnet is not positioned properly
  while((magnetStatus & 32) != 32)  {
    magnetStatus = 0; //reset reading

    Wire.beginTransmission(0x36); //connect to the sensor
    Wire.write(0x0B); //figure 21 - register map: Status: MD ML MH
    Wire.endTransmission(); //end transmission
    Wire.requestFrom(0x36, 1); //request from the sensor

    while(Wire.available() == 0); //wait until it becomes available 
    magnetStatus = Wire.read(); //Reading the data after the request      
  }        
}
