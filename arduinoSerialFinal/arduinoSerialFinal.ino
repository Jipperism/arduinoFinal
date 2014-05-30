// Sweep
// by BARRAGAN <http://barraganstudio.com> 
// This example code is in the public domain.


#include <Servo.h> 

Serial.setTimeout(50);
 
Servo myservo; 
                
 
int pos = 0;
boolean firstStageUp = true;
 
void setup() 
{ 
  myservo.attach(9);
} 
 
 
void loop() 
{ 
  while (Serial.available() > 1){
    byteInput = Serial.parseFloat();
  }
  
  if (byteInput < 100){
    if(firstStageUp){
      pos += 1;
    }
    if(!firstStageUp){
      pos -= 1;
    }
  }
  
  if (byteInput => 100 || byteInput < 200){
    
  }
  
  if (byteInput => 200){
    
  }
  
  if (byteInput < 100){
    if (pos < 1 || pos > 59){
      firstStageUp = !firstStageUp;
    }
  
  
  
                                  
    myservo.write(pos);               
    delay(15);
  }
}
