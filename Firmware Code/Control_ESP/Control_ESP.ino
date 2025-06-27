#include <ArduinoJson.h>

const int freq = 10000;
const int pwmChannel = 0;
const int resolution = 8;



#define SERIAL_TX_PIN 17
#define SERIAL_RX_PIN 16


// Motor pins
#define FR_IN1 26
#define FR_IN2 27
#define FR_PWM 14
#define FL_IN1 12
#define FL_IN2 13
#define FL_PWM 2
#define RR_IN1 25
#define RR_IN2 21
#define RR_PWM 15
#define RL_IN1 33
#define RL_IN2 32
#define RL_PWM 0

// IR sensor pins
#define LEFT_IR_PIN 35      // Left IR sensor pin 
#define MIDDLE_IR_PIN 5    // Middle IR sensor pin   
#define RIGHT_IR_PIN 23     // Right IR sensor pin

int DEFAULT_SPEED = 170; 

String serialInput = "";
bool stopFlag = false;

// Motor control functions
void goForward(int speed) {
  digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, LOW); ledcWrite(FL_PWM, speed);
  digitalWrite(RL_IN1, HIGH); digitalWrite(RL_IN2, LOW); ledcWrite(RL_PWM, speed);
  digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, HIGH); ledcWrite(FR_PWM, speed);
  digitalWrite(RR_IN1, LOW); digitalWrite(RR_IN2, HIGH); ledcWrite(RR_PWM, speed);
}

void goBackward(int speed) {
  digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, HIGH); ledcWrite(FL_PWM, speed);
  digitalWrite(RL_IN1, LOW); digitalWrite(RL_IN2, HIGH); ledcWrite(RL_PWM, speed);
  digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, LOW); ledcWrite(FR_PWM, speed);
  digitalWrite(RR_IN1, HIGH); digitalWrite(RR_IN2, LOW); ledcWrite(RR_PWM, speed);
}

void goLeft(int speed) {
  digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, LOW); ledcWrite(FL_PWM, 0);
  digitalWrite(RL_IN1, LOW); digitalWrite(RL_IN2, LOW); ledcWrite(RL_PWM, 0);
  digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, HIGH); ledcWrite(FR_PWM, speed);
  digitalWrite(RR_IN1, LOW); digitalWrite(RR_IN2, HIGH); ledcWrite(RR_PWM, speed);
}

void goRight(int speed) {
  digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, LOW); ledcWrite(FL_PWM, 255);
  digitalWrite(RL_IN1, HIGH); digitalWrite(RL_IN2, LOW); ledcWrite(RL_PWM, 255);
  digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, LOW); ledcWrite(FR_PWM, speed);
  digitalWrite(RR_IN1, HIGH); digitalWrite(RR_IN2, LOW); ledcWrite(RR_PWM, speed);
}

void stopMotors() {
  digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, LOW); ledcWrite(FL_PWM, 0);
  digitalWrite(RL_IN1, LOW); digitalWrite(RL_IN2, LOW); ledcWrite(RL_PWM, 0);
  digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, LOW); ledcWrite(FR_PWM, 0);
  digitalWrite(RR_IN1, LOW); digitalWrite(RR_IN2, LOW); ledcWrite(RR_PWM, 0);
}

void setMotor(int in1, int in2, int pwm_channel, int speed) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  ledcWrite(pwm_channel, speed);
}

void stopMotor(int in1, int in2, int pwm_channel) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  ledcWrite(pwm_channel, 0);
}
void go_left(int speed){
  digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, HIGH); ledcWrite(FL_PWM, speed);
  digitalWrite(RL_IN1, LOW); digitalWrite(RL_IN2, HIGH); ledcWrite(RL_PWM, speed);
  digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, HIGH); ledcWrite(FR_PWM, 255);
  digitalWrite(RR_IN1, LOW); digitalWrite(RR_IN2, HIGH); ledcWrite(RR_PWM, 255);
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("ESP32 Booting...");

 Serial1.begin(115200, SERIAL_8N1, SERIAL_RX_PIN, SERIAL_TX_PIN);



  // Motor pins setup
  pinMode(FL_IN1, OUTPUT); pinMode(FL_IN2, OUTPUT);
  pinMode(FR_IN1, OUTPUT); pinMode(FR_IN2, OUTPUT);
  pinMode(RL_IN1, OUTPUT); pinMode(RL_IN2, OUTPUT);
  pinMode(RR_IN1, OUTPUT); pinMode(RR_IN2, OUTPUT);

  // PWM setup
  ledcAttachChannel(FR_PWM, freq, resolution, pwmChannel);
  ledcAttachChannel(FL_PWM, freq, resolution, pwmChannel);
  ledcAttachChannel(RR_PWM, freq, resolution, pwmChannel);
  ledcAttachChannel(RL_PWM, freq, resolution, pwmChannel);

  stopMotors();

  // IR sensor pins setup
  pinMode(LEFT_IR_PIN, INPUT);
  pinMode(MIDDLE_IR_PIN, INPUT);
  pinMode(RIGHT_IR_PIN, INPUT);
}

void loop() {
  if (Serial.available() > 0) {
    serialInput = Serial.readStringUntil('\n');
    serialInput.trim();
    Serial.print("Received Command: ");
    Serial.println(serialInput);
  
    if (serialInput.equalsIgnoreCase("stop")) {
      stopMotors();
      stopFlag = true;
      Serial.println("Motors stopped.");
    } 
    else if (serialInput.equalsIgnoreCase("resume")) {
      stopFlag = false;
      Serial.println("Motors resumed.");
    }
    else if (serialInput.startsWith("speed ")) {
      // Extract the speed value from the command
      String speedStr = serialInput.substring(6);
      speedStr.trim();
      int newSpeed = speedStr.toInt();
      
      
      
      if (newSpeed >= 0 && newSpeed <= 255) {
        DEFAULT_SPEED = newSpeed;
        Serial.print("Default speed set to: ");
        Serial.println(DEFAULT_SPEED);
      } else {
        Serial.println("Invalid speed value! Please enter a value between 0 and 255.");
      }
    }
  }

  if (stopFlag) return;
  
  int LeftIR = !digitalRead(LEFT_IR_PIN);       // Read left IR sensor
  int MiddleIR = !digitalRead(MIDDLE_IR_PIN);   // Read middle IR sensor
  int RightIR = !digitalRead(RIGHT_IR_PIN);     // Read right IR sensor
  

Serial1.printf("%d,%d,%d\n", LeftIR, MiddleIR, RightIR);
  


  // Use the current default speed
  int speed = DEFAULT_SPEED;
  
  // Motor control based on IR sensors
  if ((LeftIR == 1 && MiddleIR == 1 && RightIR == 1) || (LeftIR == 0 && MiddleIR == 1 && RightIR == 0)) {
    goBackward(speed);
    delay(2000);
    go_left(speed);
    delay(1500);
    Serial.println("Moving Backward");
  } else if (LeftIR == 0 && MiddleIR == 0 && RightIR == 0) {
    goForward(speed);
    Serial.println("Moving Forward");
  } else if ((LeftIR == 0 && MiddleIR == 1 && RightIR == 1) || (LeftIR == 0 && MiddleIR == 0 && RightIR == 1)) {
    go_left(speed);
    Serial.println("Turning Left");
  } else if ((LeftIR == 1 && MiddleIR == 0 && RightIR == 0) || (LeftIR == 1 && MiddleIR == 1 && RightIR == 0)) {
    goRight(speed);
    Serial.println("Turning Right");
  } else {
    goBackward(speed);
    delay(2000);
    go_left(speed);
    delay(1500);
    Serial.println("Stopped - No clear direction");
  }

  Serial.printf("Left: %d | Mid: %d | Right: %d | Speed: %d ", 
                LeftIR, MiddleIR, RightIR, speed);

  delay(10); // Small delay to prevent flooding the serial monitor
}