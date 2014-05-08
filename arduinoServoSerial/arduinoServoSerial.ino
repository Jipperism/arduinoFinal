// Sweep
// by BARRAGAN <http://barraganstudio.com> 
// This example code is in the public domain.


#include <Servo.h>
//#include <Stream.h>
 
Servo myservo;  // create servo object to control a servo 
                // a maximum of eight servo objects can be created 
 
int pos = 0;    // variable to store the servo position
int time = 15;
int tempDelay = 15;
 
void setup() 
{ 
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object 
  Serial.begin(9600);
} 
 
 
void loop() 
{ 
  /*if(Serial.available() > 1){
    time = Serial.read();
    /*if (tempDelay >= 9 && tempDelay <=20){
      time = tempDelay;
    }
  }*/
  time = tempDelay;
  
  for(pos = 0; pos < 180; pos += 1)  // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    myservo.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(time);
    time = tempDelay;
    while (Serial.available()) {
      tempDelay = (int)Serial.read();
    }
          // waits 15ms for the servo to reach the position 
  } 
  for(pos = 180; pos>=1; pos-=1)     // goes from 180 degrees to 0 degrees 
  {                                
    myservo.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(time);
    time = tempDelay;
    while (Serial.available()) {
      tempDelay = (int)Serial.read();
    }    // waits 15ms for the servo to reach the position 
  } 
}
