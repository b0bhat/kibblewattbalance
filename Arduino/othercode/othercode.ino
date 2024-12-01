void old_force_mode() {
  if(!forceMode){
    pinMode(COILAP, OUTPUT);
    pinMode(COILAN, OUTPUT);
    analogWrite(COILAP, 0);
    analogWrite(COILAN, 0);
    delay(100);
    forceMode = true;
  }

  double rotation_radians = degAngle * (PI / 180.0);
  pos = rotation_radians * ARM_LENGTH;

  // assuming pos is being updated
  if(fabs(degAngle-calibratedAngle) <= 0.1){ // Position is balanced
    update_mass();
    Serial.print("Measured mass: ");
    Serial.println(mass);
    //digitalWrite(CALIBLED, HIGH);
  } else {
    Serial.print("pushing: ");
    Serial.println(pos);
    Serial.println(duty_cycle);
    int16_t new_duty_cycle = (duty_cycle + (-pos * 500));
    if (new_duty_cycle > 255) {
      new_duty_cycle = 255;
    }
    else if (new_duty_cycle < 0){
      new_duty_cycle = 0;
    }
    duty_cycle = new_duty_cycle;
    analogWrite(COILAP, duty_cycle);
    //digitalWrite(CALIBLED, LOW);
  }

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
  //Serial.println(correctedAngle, 3); //print the corrected/tared angle  
}



void get_current_current() {
  int sensorValue = analogRead(A0);
  float voltage = 1000 * sensorValue * (5.0 / 1023.0);
  float current = voltage/(coil1_r+res1);
  Serial.println(current,3);
  delay(10);
}
