//********************-Stepper and Servo Includes-********************
#include <AccelStepper.h>

//********************-Odrive Settings-********************
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }
HardwareSerial& odrive_serial = Serial2;
ODriveArduino odrive(odrive_serial);

//********************-Stepper Settings-********************
#define angle1_step 0
#define angle2_step -1550
#define angle3_step -3700
#define angle4_step -5800
#define angle5_step -8500
AccelStepper stepper1 = AccelStepper(1, 7, 8);
AccelStepper stepper2 = AccelStepper(1, 4, 5);

//********************-Motor Pins-********************
int motor1_pin_1 = 33;
int motor1_pin_2 = 31;
int motor1_pin_pwm = 2;
int motor2_pin_1 = 29;    
int motor2_pin_2 = 27;    
int motor2_pin_pwm = 3;   

int Servo_pin_k1 = 13;           // ESC - 1
int Servo_pin_k2 = 12;           // ESC - 2
int esc_speed_k1 = 1300;         // ESC - 1 Speed 0 --> 1500
int esc_speed_k2 = 1300;         // ESC - 1 Speed 0 --> 1500
bool k = 0;                      // ESC Button
int catapult_angle = 0;
int catapult_rotate = 0;
unsigned long shalgah=0;      
unsigned long myTime;
float spd = 0.0, spd1 = 0.0;
int butt[12];
int i;

void setup() {  
  pinMode(motor1_pin_1, OUTPUT);
  pinMode(motor1_pin_2, OUTPUT);
  pinMode(motor1_pin_pwm, OUTPUT);
  pinMode(motor2_pin_1, OUTPUT);
  pinMode(motor2_pin_2, OUTPUT);
  pinMode(motor2_pin_pwm, OUTPUT);  

  Serial.begin(57600);
  Serial1.begin(57600);
  Serial2.begin(57600);
  Serial3.begin(57600);

  stepper1.setAcceleration(300000);
  stepper1.setSpeed(100);
  stepper2.setAcceleration(300000);
  stepper2.setSpeed(100);  
  odrive_serial.begin(115200);
}

unsigned int control = 0;
unsigned int s0 = 0, s1 = 6, s2 = 0, s3 = 0;
unsigned int sdata[6];
unsigned int con[8];

void serialEvent1() {
  s0 = Serial1.read();
  if (s0 == 0xff) {
    s1 = 0;
    s3 = 1;
  } else {
    if (s3 == 1) {
      sdata[s1] = s0;
      s1++;
      if (s1 == 2) {
        s1 = 0;
        s2 = 1;
        s3 = 0;
      }
    }
  }
}

void motor_1_stop() {
  digitalWrite(motor1_pin_1, HIGH);
  digitalWrite(motor1_pin_2, HIGH);
  analogWrite(motor1_pin_pwm, 0);
}

void motor_1_active() {
  digitalWrite(motor1_pin_1, LOW);
  digitalWrite(motor1_pin_2, HIGH);
  analogWrite(motor1_pin_pwm, 128);
}

void motor_2_stop() {
  digitalWrite(motor2_pin_1, HIGH);
  digitalWrite(motor2_pin_2, HIGH);
  analogWrite(motor2_pin_pwm, 0);
}

void motor_2_active() {
  digitalWrite(motor2_pin_1, LOW);
  digitalWrite(motor2_pin_2, HIGH);
  analogWrite(motor2_pin_pwm, 128);
}

//********************-Step By Step Settings-********************
int old_button1 = 0;
int old_button2 = 0;
int new_button1 = 0;
int new_button2 = 0;
boolean btn1_state = 0;
boolean btn2_state = 0;
uint8_t btn_counter = 0;

void loop() {
  myTime = millis();  
  if (s2 == 1) {   
    
    s2 = 0;
    con[0] = sdata[0];
    con[1] = sdata[1];
    shalgah = myTime;
    
    if(bitRead(con[1], 1)) {
      motor_2_active(); 
    } else if (bitRead(con[0], 4)) {
      if (k == 0) {
        spd = spd + 10;
        k = 1;
      }
    } else if (bitRead(con[0], 6)) {
      if (k == 0) {
        spd = spd - 5;
        k = 1;
      }  
    } else if (bitRead(con[1], 6)) {
      catapult_angle=0;
    } else if (bitRead(con[1], 3)) {
      catapult_angle = catapult_angle - 20;
    } else if (bitRead(con[1], 0)) {
      catapult_angle = catapult_angle + 20;
    } else if (bitRead(con[0], 2)) {
      //catapult_angle=-500;
    } else if (bitRead(con[0], 0)) {
      //catapult_angle=+500;
    } else  if (bitRead(con[0], 7)) {
      catapult_rotate = catapult_rotate + 20;
    } else  if (bitRead(con[0], 5)) {
      catapult_rotate = catapult_rotate - 20;
    } else {
      k=0;
      motor_2_stop();
    }

    if (spd < 0) {spd = 0;}
    if (spd > 50) {spd = 50;}
   
//********************-Button Counter Settings-********************
    new_button2 = bitRead(con[0], 0);
    if(old_button2 == 0 && new_button2 == 1) {    
      if(btn2_state == 0) { 
        btn_counter = btn_counter+1;
        btn2_state = 1;
      } else {
        btn2_state = 0;          
      }
    }
    new_button1 = bitRead(con[0], 2);
    if(old_button1 == 0 && new_button1 == 1) {
      if(btn1_state == 0) {
        btn_counter = btn_counter-1;
        btn1_state = 1;
      } else {
        btn_counter = btn_counter-1;
      }
    }

    switch (btn_counter) {
     case 1:
          catapult_angle = angle1_step;
       break;
     case 2:
          catapult_angle = angle2_step;
       break;
     case 3:
          catapult_angle = angle3_step;
       break;
     case 4:
          catapult_angle = angle4_step;
       break;
     case 5:
          catapult_angle = angle5_step;
       break;
     default:
           //ktapult_ontsog=0;
       break;
    }

    if (btn_counter<0) {btn_counter = 0;}
    if (btn_counter>5) {btn_counter = 5;}

    spd1 = spd * -1;
    odrive.SetVelocity(0, spd1);
    odrive.SetVelocity(1, spd);
    delay(5);
        
    stepper1.moveTo(catapult_angle);
    stepper1.runToPosition();
    stepper2.moveTo(catapult_rotate);
    stepper2.runToPosition();
    old_button2 = new_button2;
    old_button1 = new_button1;
  }
//  Serial.print(con[0],BIN);
//  Serial.print("    ");
//  Serial.print(con[1],BIN);
//  Serial.println("   ");
}
