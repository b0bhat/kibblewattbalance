#include <SoftPWM.h>
#include <SPI.h>
#include <Wire.h>

#define COILAP A0
#define COILAN A1
#define COILAGND A2

const byte CS_pin = 10;
uint16_t command = 0b1111111111111111;

float coil1_r = 1700.0;
float res1 = 9850.0;

bool forceMode = false;

float pos;

int lowbyte; //raw angle 7:0
word highbyte; //raw angle 7:0 and 11:8
int rawAngle; //final raw angle 
float degAngle;
int magnetStatus = 0;

float calibratedAngle = 7.4;

uint8_t duty_cycle = 0;

void setup() {
  //Serial.begin(9600);
  Serial.begin(9600);
  Wire.begin(); //start i2C  
	Wire.setClock(800000L);
  checkMagnetPresence();
  readRawAngle();
}

void force_mode() {
  if(!forceMode){
    pinMode(COILAP, OUTPUT);
    pinMode(COILAN, OUTPUT);
    pinMode(COILAGND, INPUT);
    digitalWrite(COILAN, LOW);
    SoftPWMSet(COILAP, 0);
    SoftPWMSetFadeTime(COILAP, 10, 10);
    // digitalWrite(COILBP, LOW);
    // digitalWrite(COILBN, LOW);
    delay(100);
    forceMode = true;
  }

  // assuming pos is being updated
  if(fabs(degAngle) <= 0.05){ // Position is balanced
    //update_mass();
    Serial.print("Measured mass: ");
    Serial.println(mass);
    //digitalWrite(CALIBLED, HIGH);
  } else {
    int16_t new_duty_cycle = (duty_cycle + (-pos * 500));
    if (new_duty_cycle > 255) {
      new_duty_cycle = 255;
    }
    else if (new_duty_cycle < 0){
      new_duty_cycle = 0;
    }
    duty_cycle = new_duty_cycle;
    // Serial.print("Duty Cycle: ");
    // Serial.println(duty_cycle);
    SoftPWMSet(COILAP, duty_cycle);
    digitalWrite(CALIBLED, LOW);
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

void correctAngle() {
  //recalculate angle
  float correctedAngle = degAngle - calibratedAngle; //this tares the position

  if(correctedAngle < 0) { //if the calculated angle is negative, we need to "normalize" it
    correctedAngle = correctedAngle + 360; //correction for negative numbers (i.e. -15 becomes +345)
  } else if (correctedAngle >= 360) {
    correctedAngle = correctedAngle - 360;
  } else {
    //do nothing
  }
  Serial.println(correctedAngle, 3); //print the corrected/tared angle  
}

void get_current_current() {
  int sensorValue = analogRead(A0);
  float voltage = 1000 * sensorValue * (5.0 / 1023.0);
  float current = voltage/(coil1_r+res1);
  Serial.println(current,3);
  delay(10);
}

double get_current(){
  double current = (((duty_cycle / 255.0) * 100.0) * 5.0)/83.1;
  return current;
}

void update_mass(){
  double bl = get_avg_bl();
  double i = get_current();
  mass = bl * (i/GRAVITY);
}

void get_avg_bl() {
  return 4;
}

void loop() {
  //getcurrent();
  readRawAngle();
  correctAngle();
}

void checkMagnetPresence() {  
  //This function runs in the setup() and it locks the MCU until the magnet is not positioned properly

  while((magnetStatus & 32) != 32) //while the magnet is not adjusted to the proper distance - 32: MD = 1
  {
    magnetStatus = 0; //reset reading

    Wire.beginTransmission(0x36); //connect to the sensor
    Wire.write(0x0B); //figure 21 - register map: Status: MD ML MH
    Wire.endTransmission(); //end transmission
    Wire.requestFrom(0x36, 1); //request from the sensor

    while(Wire.available() == 0); //wait until it becomes available 
    magnetStatus = Wire.read(); //Reading the data after the request

    //Serial.print("Magnet status: ");
    //Serial.println(magnetStatus, BIN); //print it in binary so you can compare it to the table (fig 21)      
  }        
}
