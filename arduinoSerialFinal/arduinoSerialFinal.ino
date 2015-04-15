// Servo software for interactive art project using arduino and openframeworks
// Written by Jip Stavenuiter for Guust van Uden
// Feel free to use/modify as desired

#include <Servo.h> 

Servo myservo; 
                
int pos = 0;
int stage = 0;
int inByte = 0;
boolean going_up = false;

// Configure borders of different stages
int lower_borders[] = {0, 0, 0};
int upper_borders[] = {60, 120, 179};

// Number of steps within one stage. This is also the maximum amount
// of rotation per cycle.
int stage_steps = 2;
int per_step =  85 / stage_steps;


void setup() {
    Serial.begin(9600);
    myservo.attach(9);
    Serial.setTimeout(25);
}


void loop() {
    // Read incoming data from the serial port.
    if (Serial.available() > 0) {inByte = (int)Serial.read();}
    
    // Determine the stage ( 0, 1, 2).
    stage = determine_stage(inByte);

    pos = set_position(stage, inByte);
    pos = constrain(pos, 0, 179);
                                  
    myservo.write(pos);               
    delay(15);
}


int set_position(int input_stage, int input_byte) {
    set_flags(input_stage); 

    int output_pos = calculate_difference(input_byte);

    if (going_up){
        return pos + output_pos;
    } else {
        return pos - output_pos;
    }
}


int determine_stage(int inputByte) {
    // Determine the stage (0,1,2). 0 is furthest away.
    int result_stage = inputByte / 85;
    result_stage = constrain(result_stage, 0, 2);
    return result_stage;
}


void set_flags(int input_stage) {
    // Set going_up flag to true if the current position is below the border
    // of the stage, or false when it is above the upper border.
    if (pos <= lower_borders[input_stage]){
        going_up = true;
    } else if (pos >= upper_borders[input_stage]){
        going_up = false;
    }
}


int calculate_difference(int input_byte) {
    int phase = input_byte % 85;
    int result = phase / per_step;
    result += 1;
    return result;
}
