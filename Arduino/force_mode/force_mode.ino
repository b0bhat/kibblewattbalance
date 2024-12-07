#include <SPI.h>
#include <Wire.h>

const byte CS_pin = 10;
uint16_t command = 0b1111111111111111;

float coil1_r = 1700.0;
float res1 = 9850.0;
float c_value = 1.5;

bool forceMode = false;
bool velocityMode = false;
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

// velocity
bool coilBstate = true;

uint8_t duty_cycle = 0;

// input
char receivedChar;
boolean newData = false;


void setup() {
  //Serial.begin(9600);
  Serial.begin(9600);
  Serial.println("starting...");
  // while (!Serial) {
  //   ;
  // }
}

// set to digital pins instead
#define LEDRED 3
#define COILBP 5
#define COILBN 6

#define COILAP 9
#define COILAN 10
#define LEDGREEN 11

#define ON 70
#define OFF 0

void begininit() {
  // setup hall effect sensor
  Wire.begin(); //start i2C  
	Wire.setClock(800000L);
  checkMagnetPresence();
  readRawAngle();
  calibratedAngle = degAngle;

  // Serial.println("initialized");
  // Serial.print("calibrated angle: ");
  pinMode(LEDGREEN, OUTPUT);
  Serial.println(degAngle);

  open_state();
}

void force_mode_setup() {
  pinMode(COILAP, OUTPUT);
  pinMode(COILAN, OUTPUT);
  analogWrite(COILAP, OFF);
  analogWrite(COILAN, OFF);
  
  used_state();
  
  delay(500);
  forceMode = true;
}

void force_mode() {
  // force mode setup should be complete
  // assuming pos is being updated
  //Serial.println(fabs(degAngle-calibratedAngle));
  if(fabs(degAngle-calibratedAngle) <= 0.08) {
    update_mass();
  } else if(duty_cycle < 255){ // Position is not yet balanced
    //Serial.print("pushing: ");
    //Serial.println(duty_cycle);
    analogWrite(COILAP, duty_cycle);
    duty_cycle  = duty_cycle + 3;
    //duty_cycle = max(0, min(duty_cycle, 255));
    delay(300);
    readRawAngle();
    Serial.print("Angle Difference:");
    Serial.print(fabs(degAngle-calibratedAngle), 5);
    Serial.print(",");
    Serial.print("Voltage:");
    Serial.println(get_voltage(), 5);
    //digitalWrite(CALIBLED, LOW);
  } else {
    delay(100);
    Serial.println("max voltage reached");
    update_mass();
  }

}

void open_state() {
  analogWrite(LEDGREEN, ON);
  analogWrite(LEDRED, OFF);
}

void used_state() {
  analogWrite(LEDGREEN, OFF);
  analogWrite(LEDRED, ON);
}

void calibrate() {
  pinMode(COILAP, OUTPUT);
  pinMode(COILAN, OUTPUT);

  used_state();

  analogWrite(COILAP, 200);
  delay(6000);

  readRawAngle();
  c_value = fabs(degAngle-calibratedAngle);
  Serial.println("calibrated diff: " + String(c_value));
  analogWrite(COILAP, OFF);
  degAngle = 0;
  
  open_state();

}

// void velocity_mode_setup() {
//   analogWrite(COILAP, OFF);
//   pinMode(COILAP, OUTPUT);
//   pinMode(COILAN, OUTPUT);
//   pinMode(COILAP, INPUT);
//   pinMode(COILAN, INPUT);
//   delay(500);
//   forceMode = true;
// }

// void velocity_mode() {
//   int t1, t2;
//   double p1, p2, v1, v2;
  
//   drive_coil_b();
//   t1 = millis();
//   v1 = read_voltage();
//   p1 = degAngle;
//   double vel = (p2 - p1) * 1000 / (t2 - t1);
// }

double read_voltage() {
  double adc_val = (double)analogRead(COILAP) - (double)analogRead(COILAN);
  return (adc_val * 5.0)/ 1024.0;
}

// double bl[BL_ARRAY_SIZE];
// int bl_index = 0;

// void drive_coil_b() {
//   if (millis() - prevCoilBSwitch < coilBswitchInterval) { // If not time to switch exit
//     return;
//   }

//   if (coilBstate) {
//     analogWrite(COILBP, OFF);
//     analogWrite(COILBN, ON);
//   } else {
//     analogWrite(COILBP, ON);
//     analogWrite(COILBN, OFF);
//   }
//   prevCoilBSwitch = millis();
// }


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
}

double get_voltage() {
  return (duty_cycle / 255.0) * 5.0;
}

double get_current(){
  double current = get_voltage()/1800; // I=V/R
  return current*100;
}

void update_mass(){
  double i = get_current();
  mass = (sqrt(c_value)-0.3) * 100 * (i/GRAVITY);

  Serial.print("Measured mass (g): ");
  Serial.println(mass);
  resetState();
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
  analogWrite(LEDRED, 0);
  analogWrite(LEDGREEN, 0);
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

  if (receivedChar == 'c') {
    calibrate();
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
    Serial.print("|");

    Wire.beginTransmission(0x36); //connect to the sensor
    Wire.write(0x0B); //figure 21 - register map: Status: MD ML MH
    Wire.endTransmission(); //end transmission
    Wire.requestFrom(0x36, 1); //request from the sensor

    while(Wire.available() == 0); //wait until it becomes available 
    magnetStatus = Wire.read(); //Reading the data after the request      
  }
  Serial.print("\n");    
}
