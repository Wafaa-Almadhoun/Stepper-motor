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
    2. 1  NEMA 17 bipolar stepper
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

### 2. Bipolar Stepper with L293D Motor Driver IC .[see here ](https://github.com/Wafaa-Almadhoun/Stepper-motor-using-Arduino-UNO-R3-/blob/main/Bipolar%20Stepper%20with%20L293D%20Motor%20Driver%20IC.pdsprj)
![1](https://user-images.githubusercontent.com/64277741/179328636-268173e6-09b8-46fb-9431-1dfe2eae640f.PNG)
Figure (7): step one revolution in the other direction ("counterclockwise")

 ![2](https://user-images.githubusercontent.com/64277741/179328701-3dee3532-ada8-4ae9-abdd-f15dcee8762f.PNG)
Figure (8): step one revolution in one direction ("clockwise")

#### The code 

// Include the Arduino Stepper Library
#include <Stepper.h>

// Number of steps per output rotation NEMA 17

const int stepsPerRevolution = 200; 

// Create Instance of Stepper library

Stepper myStepper(stepsPerRevolution, 12, 11, 10, 9);


void setup()
{
  // set the speed at 20 rpm:
  
  myStepper.setSpeed(20);
  
}

void loop() 
{
  // step one revolution in one direction:
  
  myStepper.step(stepsPerRevolution);
  
  delay(1000);

  // step one revolution in the other direction:
  
  myStepper.step(-stepsPerRevolution);
  
  delay(1000);
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


