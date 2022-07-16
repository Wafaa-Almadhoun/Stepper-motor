# Stepper-motor-using-Arduino-UNO-R3-



## Table of contents
* [Introduction](#Introduction)
* [Technologies](#technologies)
* [Components required](#Components-required)
* [Connections](#Connections)
* [Block diagram & simulation ](#Block-diagram-&-simulation)



## Introduction
Stepper motors are an ideal choice for accurately moving and positioning mechanical devices. Using techniques like microstepping the position of the motor shaft can be controlled with a great deal of precision, stepper Motors are used in a wide variety of devices ranging from 3D printers and CNC machines to DVD drives, heating ducts, and even analog clocks , stepper motors are DC motors that rotate in precise increments or “steps”.

  ,We will cover several topics : 👍 


 1. Unipolar Stepper with ULN2003.
 2. Bipolar Stepper with L293D Motor Driver IC.
 3. servo motor with PCA9685 module rotate from 0 to 90 degrees and to 120 degree . 
 4. servo motor with PCA9685 module  Controlled by using Potentiometer. 


## Technologies
Project is created with:
* Arduino IDE 1.8.19 [To Downloud](https://www.arduino.cc/en/software)
* Proteus [To Downloud](https://www.labcenter.com/simulation/)
	
## Components required
### 1. Unipolar Stepper with ULN2003 
    1. Arduino UNO
    2. 1 – 28BYJ-48 Unipolar Stepper
    3. jumper wirs
    4. driver board ULN2003
    5. bettrey  5 and 12 volt 
    6. breadboard
### 2. Bipolar Stepper with L293D Motor Driver IC
    1. Arduino UNO
    2. 1 bipolar stepper
    3. jumper wirs
    4. L293D Motor Driver IC
    5. bettrey  5 and 12 volt 
    6. breadboard

## Connections

### 1. Unipolar Stepper with ULN2003

     connecting ULN2003 pin1 to pin 8 in Ardunio
     connecting ULN2003 pin2 to pin 9 in Ardunio
     connecting ULN2003 pin3 to pin 10 in Ardunio
     connecting ULN2003 pin4 to pin 11 in Ardunio
     connecting ULN2003 pin16 to pin1 in stepper
     connecting ULN2003 pin15 to pin2 in stepper
     connecting ULN2003 pin14 to pin3 in stepper
     connecting ULN2003 pin13 to pin4 in stepper
     connecting ULN2003 pin9 and the 2 2VDD pin in stepper motor to 12v battery 
     Connect ground to ground
     
 ### 2. Bipolar Stepper with L293D Motor Driver IC
 
    connecting 5V output on Arduino to the Vcc2 & Vcc1 pins
    
    Connect ground to ground.
    
    connect the input pins(IN1, IN2, IN3 and IN4) of the L293D IC to 
    
    four digital output pins(12, 11, 10 and 9) on Arduino
    0ne coil of stepper moter connecting to Out1 & Out2 and the anthor coil connecting to Out3 & Out4
    
 
 
 ### 3. servo motor with PCA9685 module rotate from 0 to 90 degrees and to 120 degree 
 
     connected SCL in Ardunio to SCL in PCA9685 
  
     connected SDA in Ardunio to SDA in PCA9685
  
     connected GND in Ardunio to GND in PCA9685
  
     2-pin screw connector at the top for the servo with 5v power supply
  
     connected 5v in Ardunio to VCC in PCA9685
  
     connected servo motor to outputs 0 in PCA9685
     
###  4. servo motor with PCA9685 module  Controlled by using Potentiometer
    
     connected SCL in Ardunio to SCL in PCA9685 
  
     connected SDA in Ardunio to SDA in PCA9685
  
     connected GND in Ardunio to GND in PCA9685
  
     2-pin screw connector at the top for the servo with 5v power supply
  
     connected 5v in Ardunio to VCC in PCA9685
  
     connected servo motor to outputs 0 in PCA9685
     
     connected Potentiometer signal to A0 in Arduino
     
     connected Potentiometer GND to GND in Arduino
     
     connected Potentiometer VDD to 5v in Arduino
     
## Block diagram & simulation
### 1. Unipolar Stepper with ULN2003 . [see here](https://github.com/Wafaa-Almadhoun/Stepper-motor-using-Arduino-UNO-R3-/blob/main/stepper%20using%20ULN2003.pdsprj)
##### Slow - 4-step CW sequence to observe lights on driver board
![1](https://user-images.githubusercontent.com/64277741/179306291-f9684758-deaf-4828-9520-757a142ba537.PNG)
Figure (1): Stepper Motor at 90 degree after 1-step CW sequence
![2](https://user-images.githubusercontent.com/64277741/179307189-82e1089f-4cbb-403a-b78c-a4c990c24522.PNG)
Figure (2): Stepper Motor at 180 degree after 2-step CW sequence
![3](https://user-images.githubusercontent.com/64277741/179307421-bbcf698d-139f-4d30-aae9-9546c057fb68.PNG)
Figure (3): Stepper Motor at 270 degree after 3-step CW sequence
![4](https://user-images.githubusercontent.com/64277741/179307644-0f9d39bf-591d-45a1-b9c4-d38ec8528d7d.PNG)
Figure (4): Stepper Motor at 342 degree after 4-step CW sequence
##### Rotate CW 1/2 turn slowly
![5](https://user-images.githubusercontent.com/64277741/179308867-85dbccdc-5070-4164-82c7-9776d4b09fc9.PNG)
Figure (5): Rotate CW 1/2 turn slowly
##### Rotate CCW 1/2 turn quickly
![6](https://user-images.githubusercontent.com/64277741/179309329-1eed85e5-3f0d-48a3-b2be-3d9a40e71869.PNG)
Figure (6): Rotate CCW 1/2 turn quickly

#### The Code 
 Demonstrates 28BYJ-48 Unipolar Stepper with ULN2003 Driver
 
  Uses Arduino Stepper Library
 
//Include the Arduino Stepper Library

#include <Stepper.h>
 
// Define Constants
 
// Number of steps per internal motor revolution 

const float STEPS_PER_REV = 32; 
 
//  Amount of Gear Reduction

const float GEAR_RED = 64;
 
// Number of steps per geared output rotation

const float STEPS_PER_OUT_REV = STEPS_PER_REV * GEAR_RED;
 
// Define Variables
 
// Number of Steps Required

int StepsRequired;
 
// Create Instance of Stepper Class

// Specify Pins used for motor coils

// The pins used are 8,9,10,11 

// Connected to ULN2003 Motor Driver In1, In2, In3, In4 

// Pins entered in sequence 1-3-2-4 for proper step sequencing
 
Stepper steppermotor(STEPS_PER_REV, 8, 10, 9, 11);
 
void setup()
{

// Nothing  (Stepper Library sets pins as outputs)

}
 
void loop()
{

  // Slow - 4-step CW sequence to observe lights on driver board
  
  steppermotor.setSpeed(1);    
  
  StepsRequired  =  4;
  
  steppermotor.step(StepsRequired);
  
  delay(2000);
 
   // Rotate CW 1/2 turn slowly
   
  StepsRequired  =  STEPS_PER_OUT_REV / 2; 
  
  steppermotor.setSpeed(100);   
  
  steppermotor.step(StepsRequired);
  
  delay(1000);
  
  // Rotate CCW 1/2 turn quickly
  
  StepsRequired  =  - STEPS_PER_OUT_REV / 2;   
  
  steppermotor.setSpeed(700);  
  
  steppermotor.step(StepsRequired);
  
  delay(2000);
 
}

### 2. servo motor Controlled by using Potentiometer simulated with TINKERCAD circuit .[see here ](https://www.tinkercad.com/things/bbagRhCJEr8-servo-motor-controlled-by-using-potentiometer/editel)

![servo motor Controlled by using Potentiometer](https://user-images.githubusercontent.com/64277741/179123789-863c1598-059b-42cd-8c3f-d3abc94e5751.png)
Figure (3): Servo Motor at initial value (0 degree)

After changing the value of Potentiometer 
![servo motor Controlled by using Potentiometer (1)](https://user-images.githubusercontent.com/64277741/179124038-f83b191b-ad75-4a50-9389-99e6ef5e4fe5.png)
Figure (4): Servo Motor at 180 degrees

After changing the value of Potentiometer 

![servo motor Controlled by using Potentiometer (2)](https://user-images.githubusercontent.com/64277741/179124144-9b929f91-288c-4dbe-83be-e01724491ac1.png)
Figure (5): Servo Motor at 60 degree 
#### The code 

#include <Servo.h>

Servo myservo;  // create servo object to control a servo

int potpin = 0;  // analog pin used to connect the potentiometer

int val;    // variable to read the value from the analog pin

void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
}

void loop() {

  val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
  
  val = map(val, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
  
  myservo.write(val);                  // sets the servo position according to the scaled value
  
  delay(15);                           // waits for the servo to get 
  
}


### servo motor with PCA9685 module rotate from 0 to 90 degrees and to 120 degree  
![Untitled Sketch 2_bb](https://user-images.githubusercontent.com/64277741/179159046-d9a5dfe8-6d33-43f3-934e-13ba9179969a.png)

#### The Code 
The sketch makes use of the Adafruit PWM Servo Driver Library which you will need to install to make this work.  It can be installed from the Library Manager in your Arduino IDE.

#include <Wire.h>

#include <Adafruit_PWMServoDriver.h>


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define MIN_PULSE_WIDTH 650

#define MAX_PULSE_WIDTH 2350

#define DEFAULT_PULSE_WIDTH 1500

#define FREQUENCY 50


int servonum =0;

void setup() 
{ 

Serial.begin(9600);

Serial.println("16 channel Servo test!");

pwm.begin();

pwm.setPWMFreq(FREQUENCY);

}
int pulseWidth(int angle)
{
int pulse_wide, analog_value;

pulse_wide = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);

analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);

Serial.println(analog_value);

return analog_value;

}

void loop() {

pwm.setPWM(0, 0, pulseWidth(0));

delay(1000);

pwm.setPWM(0, 0, pulseWidth(90));

delay(500);

pwm.setPWM(0, 0, pulseWidth(120));

delay(1000);

}

### 4. servo motor with PCA9685 module  Controlled by using Potentiometer

![Sketch  servo motor with PCA9685 module Controlled by using Potentiometer 2_bb](https://user-images.githubusercontent.com/64277741/179161772-121b9566-95d1-4b0b-9e70-b505945ecf0c.png)

#### The code

// Include Wire Library for I2C Communications

#include <Wire.h>
 
// Include Adafruit PWM Library

#include <Adafruit_PWMServoDriver.h>
 
#define MIN_PULSE_WIDTH       650

#define MAX_PULSE_WIDTH       2350

#define FREQUENCY             50
 
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
 
// Define Potentiometer Inputs
 
int controlA = A0;

// Define Motor Outputs on PCA9685 board
 
int motorA = 0;

void setup() 
{
  pwm.begin();
  
  pwm.setPWMFreq(FREQUENCY);
  
}
 
 
void moveMotor(int controlIn, int motorOut)

{
  int pulse_wide, pulse_width, potVal;
  
  
  // Read values from potentiometer
  
  potVal = analogRead(controlIn);
  
  
  // Convert to pulse width
  
  pulse_wide = map(potVal, 0, 1023, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  
  pulse_width = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  
  //Control Motor
  
  pwm.setPWM(motorOut, 0, pulse_width);
  
 
}
 
void loop() {
 
  //Control Motor A
  
  moveMotor(controlA, motorA);
  
}


