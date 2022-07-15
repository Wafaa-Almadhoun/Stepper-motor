# Stepper-motor-using-Arduino-UNO-R3-
electronic path week 2 task


## Table of contents
* [Introduction](#Introduction)
* [Technologies](#technologies)
* [Components required](#Components-required)
* [Connections](#Connections)
* [Block diagram & simulation ](#Block-diagram-&-simulation)



## Introduction
Stepper motors are an ideal choice for accurately moving and positioning mechanical devices. Using techniques like microstepping the position of the motor shaft can be controlled with a great deal of precision, stepper Motors are used in a wide variety of devices ranging from 3D printers and CNC machines to DVD drives, heating ducts, and even analog clocks , stepper motors are DC motors that rotate in precise increments or “steps”.

  ,We will cover several topics : 👍 


 1. Unipolar Stepper with ULN2003 .
 2. servo motor Controlled by using Potentiometer simulated with TINKERCAD circuit .
 3. servo motor with PCA9685 module rotate from 0 to 90 degrees and to 120 degree . 
 4. servo motor with PCA9685 module  Controlled by using Potentiometer. 


## Technologies
Project is created with:
* Arduino IDE 1.8.19 [To Downloud](https://www.arduino.cc/en/software)
* AUTODESK TINKERCAD [Open](https://www.tinkercad.com/)
	
## Components required
1. Arduino UNO
2. 1 servo motors MG995
3. jumper wirs
4. 1 potentiometer 10 K ohm 
5. bettrey  5 volt 
6. breadboard
7. PCA9685 module 

## Connections

### 1. servo motor rotate from 0 to 90 degrees and back simulated with TINKERCAD circuit  .
  
     5V: Power (red) to servo 
     
     Gnd: Ground (black) to servo
     
     we attach the servo object to pin 9 in arduino 
   
### 2. servo motor Controlled by using Potentiometer simulated with TINKERCAD circuit .
     5V: Power (red) to servo 
     
     Gnd: Ground (black) to servo
     
     we attach the servo object to pin 9 in arduino 
       
     5v: power from Arduino to breadboard to Potentiometer
     
     signal of Potentiometer attach to pin A0 in arduino 
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
### 1.servo motor rotate from 0 to 90 degrees and back simulated with TINKERCAD circuit . [see here](https://www.tinkercad.com/things/9Hd8sj9JjSr-servo-motor-rotate-from-0-to-90-degrees/editel)
![Cool Bigery-Uusam (3)](https://user-images.githubusercontent.com/64277741/179117203-9ee32234-7d2c-4f77-b2a4-db39a51de82f.png)
Figure (1): Servo Motor at initial value (0 degree

After 30 ms
![Cool Bigery-Uusam (2)](https://user-images.githubusercontent.com/64277741/179117381-296e4d7b-d3bb-45ee-b0f0-a619268678ff.png)
Figure (2): Servo Motor at 90 degrees 

#### The Code 
   #include <Servo.h>

   Servo myservo;  // create servo object to control a servo


   int pos = 0;    // variable to store the servo position

   void setup() {

     myservo.attach(9);  // attaches the servo on pin 9 to the servo object
   }

   void loop() {

     for (pos = 0; pos <= 90; pos += 1) { // goes from 0 degrees to 90 degrees
  
       // in steps of 1 degree
    
       myservo.write(pos);              // tell servo to go to position in variable 'pos'
    
       delay(30);                       // waits 30ms for the servo to reach the position
    
     }
  
     for (pos = 90; pos >= 0; pos -= 1) { // goes from 90 degrees to 0 degrees
  
       myservo.write(pos);              // tell servo to go to position in variable 'pos'
    
       delay(30);                       // waits 30ms for the servo to reach the position
    
     }
  
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


