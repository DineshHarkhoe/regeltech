#include <Servo.h>
#include <PID_v2.h>
#include "MPU9250.h"

int val = 0;
int oldVal = 1000;
int servoPin1 = 11;
int servoPin2 = 10;
float pitch;
int intPitch = 0;

Servo servo1;
Servo servo2;

MPU9250 mpu;
double Kp = 0.3, Ki = 0.07, Kd = 0;
PID_v2 myPID(Kp, Ki, Kd, PID::Direct);

void setup() {  
  Serial.begin(115200);
  Wire.begin();
  
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  
  servo1.writeMicroseconds(1000); // send "stop" signal to ESC.
  servo2.writeMicroseconds(1000);

  if (!mpu.setup(0x68)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
  }

  myPID.SetOutputLimits(-200, 200);
  
  myPID.Start(pitch,                // input
              0,                    // current output
              0);                   // setpoint  
   
  delay(3000); // delay to allow the ESC to recognize the stopped signal
  Serial.println("done");
}

void loop() {
  if(Serial.available() > 0){
    val = Serial.parseInt();
    //Serial.println(val);
    if (val <1000 || val > 1500){
      val = oldVal;
    }
  }
  oldVal = val;

  double pitchCorrectionFactor = myPID.Run(intPitch);
  //Serial.print("factor: ");
  //Serial.println(pitchCorrectionFactor);

  driveMotors(val + pitchCorrectionFactor, val - pitchCorrectionFactor);

  readPitch();
  Serial.println(pitch);
}

void readPitch(){
  if (mpu.update()) {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 25) {
            pitch = mpu.getPitch() - 6.8;
            intPitch = (int)pitch;
            prev_ms = millis();
        }
  }  
}

void driveMotors(int thrust1, int thrust2){
  servo1.writeMicroseconds(thrust1 - 13); // Send signal to ESC.
  servo2.writeMicroseconds(thrust2);
}