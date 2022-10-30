// Radio
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include <Servo.h>    // servo
Servo servo1;
Servo servo2;

float servo1Pos = 1500;
float servo1PosFiltered = 1500;
float servo2Pos = 1600;
float servo2PosFiltered = 1600;

#include <PID_v1.h>   // PID

double Pk0 = 0.25; 
double Ik0 = 0;
double Dk0 = 0.1;

double Setpoint0, Input0, Output0, Output0a;    // PID variables
PID PID0(&Input0, &Output0, &Setpoint0, Pk0, Ik0 , Dk0, DIRECT);    // PID Setup

double Pk1 = 1; 
double Ik1 = 0;
double Dk1 = 0.15;

double Setpoint1, Input1, Output1, Output1a;    // PID variables
PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);    // PID Setup

double Pk2 = 1; 
double Ik2 = 0;
double Dk2 = 0.15;

double Setpoint2, Input2, Output2, Output2a;    // PID variables
PID PID2(&Input2, &Output2, &Setpoint2, Pk2, Ik2 , Dk2, DIRECT);    // PID Setup

double Pk3 = 0.5; 
double Ik3 = 0;
double Dk3 = 0.1;

double Setpoint3, Input3, Output3, Output3a;    // PID variables
PID PID3(&Input3, &Output3, &Setpoint3, Pk3, Ik3 , Dk3, DIRECT);    // PID Setup

double Pk4 = 0.5; 
double Ik4 = 0;
double Dk4 = 0.1;

double Setpoint4, Input4, Output4, Output4a;    // PID variables
PID PID4(&Input4, &Output4, &Setpoint4, Pk4, Ik4 , Dk4, DIRECT);    // PID Setup


RF24 radio(9, 10); // CE, CSN
const byte addresses[][6] = {"00001", "00002"};

//**************remote control****************
struct RECEIVE_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
    int16_t menuDown;  
    int16_t Select;    
    int16_t menuUp;  
    int16_t toggleBottom;  
    int16_t toggleTop; 
    int16_t toggle1;
    int16_t toggle2;
    int16_t mode;  
    int16_t RLR;
    int16_t RFB;
    int16_t RT;
    int16_t LLR;
    int16_t LFB;
    int16_t LT;
};

RECEIVE_DATA_STRUCTURE mydata_remote;

float RLRraw = 0;
float RFBraw = 0;
float RTraw = 0;
float LLRraw = 0;
float LFBraw = 0;
float LTraw = 0;

float RLR = 0;
float RFB = 0;
float RFBa = 0;
float RT = 0;
float LLR = 0;
float LFB = 0;
float LT = 0;

float LTa = 0;
float LTb = 0;


unsigned long currentMillis;
long previousMillis = 0;    // set up timers
long interval = 10;        // time constant for timer
long previousSafetyMillis;    // timer to check the data is there or stop on safties

int stepFlag = 0;
long previousStepMillis = 0;
int stepStartFlag = 0;

int timerVar1;

#define encoder0PinA 8      // encoder 0
#define encoder0PinB 24
#define encoder1PinA 7      // encoder 1
#define encoder1PinB 6
#define encoder2PinA 5      // encoder 2
#define encoder2PinB 4
#define encoder3PinA 3      // encoder 3
#define encoder3PinB 2
#define encoder4PinA 1      // encoder 4
#define encoder4PinB 0

volatile long encoder0Pos = 0;    // encoder 0      // top carriage
volatile long encoder1Pos = 0;    // encoder 1      // right leg from back
volatile long encoder2Pos = 0;    // encoder 2      // left leg from back
volatile long encoder3Pos = 0;    // encoder 3      // left foot from back
volatile long encoder4Pos = 0;    // encoder 4      // right foot from back

float encoder0Demand;
float encoder1Demand;
float encoder2Demand;
float encoder3Demand;
float encoder4Demand;

void setup() {

    // initialize serial communication
    Serial.begin(115200);
    
    radio.begin();
    radio.openWritingPipe(addresses[0]); // 00002
    radio.openReadingPipe(1, addresses[1]); // 00001
    radio.setPALevel(RF24_PA_MIN);
    radio.startListening();

    // *********** encoders ***************

    pinMode(encoder0PinA, INPUT_PULLUP);    // encoder pins 0
    pinMode(encoder0PinB, INPUT_PULLUP);
    attachInterrupt(encoder0PinA, doEncoderA, CHANGE);
    attachInterrupt(encoder0PinB, doEncoderB, CHANGE);

    pinMode(encoder1PinA, INPUT_PULLUP);    // encoder pins 1
    pinMode(encoder1PinB, INPUT_PULLUP);
    attachInterrupt(encoder1PinA, doEncoderC, CHANGE);
    attachInterrupt(encoder1PinB, doEncoderD, CHANGE);

    pinMode(encoder2PinA, INPUT_PULLUP);    // encoder pins 2
    pinMode(encoder2PinB, INPUT_PULLUP);
    attachInterrupt(encoder2PinA, doEncoderE, CHANGE);
    attachInterrupt(encoder2PinB, doEncoderF, CHANGE);

    pinMode(encoder3PinA, INPUT_PULLUP);    // encoder pins 3
    pinMode(encoder3PinB, INPUT_PULLUP);
    attachInterrupt(encoder3PinA, doEncoderG, CHANGE);
    attachInterrupt(encoder3PinB, doEncoderH, CHANGE);

    pinMode(encoder4PinA, INPUT_PULLUP);    // encoder pins 4
    pinMode(encoder4PinB, INPUT_PULLUP);
    attachInterrupt(encoder4PinA, doEncoderI, CHANGE);
    attachInterrupt(encoder4PinB, doEncoderJ, CHANGE);


    // ******************** PWMs ****************

    pinMode(14, OUTPUT);    // axis 0 - top carriage
    pinMode(15, OUTPUT);
    pinMode(25, OUTPUT);    // axis 1 - left leg from back
    pinMode(28, OUTPUT);
    pinMode(18, OUTPUT);    // axis 2 - right leg from back
    pinMode(19, OUTPUT);
    pinMode(36, OUTPUT);    // axis 3 - left foot
    pinMode(37, OUTPUT);
    pinMode(22, OUTPUT);    // axis 4 -  right foot
    pinMode(23, OUTPUT);


    PID0.SetMode(AUTOMATIC);              
    PID0.SetOutputLimits(-180, 180);
    PID0.SetSampleTime(10);

    PID1.SetMode(AUTOMATIC);              
    PID1.SetOutputLimits(-180, 180);
    PID1.SetSampleTime(10);

    PID2.SetMode(AUTOMATIC);              
    PID2.SetOutputLimits(-180, 180);
    PID2.SetSampleTime(10);

    PID3.SetMode(AUTOMATIC);              
    PID3.SetOutputLimits(-180, 180);
    PID3.SetSampleTime(10);

    PID4.SetMode(AUTOMATIC);              
    PID4.SetOutputLimits(-180, 180);
    PID4.SetSampleTime(10);

    servo1.attach(33);
    servo2.attach(34);

    servo1.writeMicroseconds(1500);   // left leg from back  | lower to turn
    servo2.writeMicroseconds(1600);   // right leg from back | higher to turn
   
}   // end of setup

// ********************* MAIN LOOP *******************************

void loop() {  

      
        currentMillis = millis();
        if (currentMillis - previousMillis >= 10) {  // start timed event
          
            previousMillis = currentMillis;


            // check for radio data
            if (radio.available()) {
                    radio.read(&mydata_remote, sizeof(RECEIVE_DATA_STRUCTURE));  
                    previousSafetyMillis = currentMillis;   
            }

            else if (currentMillis - previousSafetyMillis >= 300) {   
                  //Serial.println("STOP");
                  mydata_remote.RFB = 512;
                  mydata_remote.RLR = 512;
                  mydata_remote.RT = 512;
                  mydata_remote.LFB = 512;
                  mydata_remote.LLR = 512;
                  mydata_remote.LT = 512;
            }            

            else {
              Serial.println("no data");              
            }     
            
            // threshold remote data
            // some are reversed based on stick wiring in remote
            RFB = thresholdStick(mydata_remote.RFB);  
            RLR = thresholdStick(mydata_remote.RLR); 
            RT = thresholdStick(mydata_remote.RT);
            LFB = thresholdStick(mydata_remote.LFB);
            LLR = thresholdStick(mydata_remote.LLR);
            LT = thresholdStick(mydata_remote.LT);

            timerVar1 = 1000+abs(RFB);      // dynamically adjust timers based on step length
            
            if (mydata_remote.toggleTop == 1 && stepFlag == 0) {      // start walking              
              previousStepMillis = currentMillis;
              stepFlag = 1;              
            }

            // *** mid position ***
            if (stepFlag == 0 && currentMillis - previousStepMillis > 3000) {
              previousStepMillis = currentMillis;
              encoder0Demand = 0;      // carriage
            }

            // *** move carriage right ***
            else if (stepFlag == 1 && currentMillis - previousStepMillis > 1700) {
              encoder0Demand = 3600;   // carriage right
              previousStepMillis = currentMillis;
              stepFlag = 2;
            }

            // *** pick up left leg ***
            else if (stepFlag == 2 && currentMillis - previousStepMillis > 1500) {
              encoder3Demand = 4500;
              previousStepMillis = currentMillis;
              stepFlag = 3;
            }

            // *** take step ***
            else if (stepFlag == 3 && currentMillis - previousStepMillis > 1500) {
              encoder1Demand = RFB * -2;
              encoder2Demand = RFB * -2;
              if (LT != 0) {
                servo1Pos = 1500+abs(LT);
                servo2Pos = 1600-abs(LT);
              } 
              else if (LT == 0) { 
                servo1Pos = 1500;
                servo2Pos = 1600;           
              }
              previousStepMillis = currentMillis;
              stepFlag = 4;
            }

            // *** put down left leg
            else if (stepFlag == 4 && currentMillis - previousStepMillis > timerVar1) {
              encoder3Demand = 0;
              previousStepMillis = currentMillis;
              stepFlag = 5;
            }

            // *** move carriage left ***
            else if (stepFlag == 5 && currentMillis - previousStepMillis > 1700) {
              encoder0Demand = -3600;  // carriage left
              previousStepMillis = currentMillis;
              stepFlag = 6;
            } 

            // *** pick up right leg ***
            else if (stepFlag == 6 && currentMillis - previousStepMillis > 1500) {
              encoder4Demand = -4500;
              previousStepMillis = currentMillis;
              stepFlag = 7;
            }  

            // *** take step ***
            else if (stepFlag == 7 && currentMillis - previousStepMillis > 1500) {
              encoder1Demand = RFB * 2;
              encoder2Demand = RFB * 2;
              if (LT != 0) {
                servo1Pos = 1500+abs(LT);
                servo2Pos = 1600-abs(LT);
              } 
              else if (LT == 0) { 
                servo1Pos = 1500;
                servo2Pos = 1600;           
              }
              previousStepMillis = currentMillis;
              stepFlag = 8;
            }  

            // *** put down right leg ***
            else if (stepFlag == 8 && currentMillis - previousStepMillis > timerVar1) {
              encoder4Demand = 0;
              previousStepMillis = currentMillis;
              stepFlag = 0;
            } 

            // control motors

            // **** carriage motor ***
            Setpoint0 = filter(encoder0Demand, Setpoint0, 60);
            Input0 = encoder0Pos;
            PID0.Compute();            

            if (Output0 > 0) {
              analogWrite(14, Output0);
              analogWrite(15, 0);
            }
            else if (Output0 < 0) {
              Output0a = abs(Output0);
              analogWrite(15, Output0a);
              analogWrite(14, 0);
            }
            else {
              analogWrite(14, 0);
              analogWrite(15, 0);
            }

            // **** right leg motor ***
            Setpoint1 = filter(encoder1Demand, Setpoint1, 30);
            Input1 = encoder1Pos;
            PID1.Compute();  

            if (Output1 > 0) {
              analogWrite(18, Output1);
              analogWrite(19, 0);
            }
            else if (Output1 < 0) {
              Output1a = abs(Output1);
              analogWrite(19, Output1a);
              analogWrite(18, 0);
            }
            else {
              analogWrite(18, 0);
              analogWrite(19, 0);
            }

            // **** left leg motor ***
            Setpoint2 = filter(encoder2Demand, Setpoint2, 30);
            Input2 = encoder2Pos;
            PID2.Compute();  

            if (Output2 > 0) {
              analogWrite(28, Output2);
              analogWrite(25, 0);
            }
            else if (Output2 < 0) {
              Output2a = abs(Output2);
              analogWrite(25, Output2a);
              analogWrite(28, 0);
            }
            else {
              analogWrite(25, 0);
              analogWrite(28, 0);
            }

            // **** left foot motor ****

            Setpoint3 = filter(encoder3Demand, Setpoint3, 5);
            Input3 = encoder3Pos;
            PID3.Compute();         

            if (Output3 > 0) {
              analogWrite(37, Output3);
              analogWrite(36, 0);
            }
            else if (Output3 < 0) {
              Output3a = abs(Output3);
              analogWrite(36, Output3a);
              analogWrite(37, 0);
            }
            else {
              analogWrite(36, 0);
              analogWrite(37, 0);
            }

             // **** Right foot motor ****

            Setpoint4 = filter(encoder4Demand, Setpoint4, 5);
            Input4 = encoder4Pos;
            PID4.Compute(); 


            if (Output4 > 0) {
              analogWrite(23, Output4);
              analogWrite(22, 0);
            }
            else if (Output4 < 0) {
              Output4a = abs(Output4);
              analogWrite(22, Output4a);
              analogWrite(23, 0);
            }
            else {
              analogWrite(22, 0);
              analogWrite(23, 0);
            }

            // *** servos ***

            servo1PosFiltered = filter(servo1Pos, servo1PosFiltered, 30);
            servo2PosFiltered = filter(servo2Pos, servo2PosFiltered, 30);

            servo1.writeMicroseconds(servo1PosFiltered);
            servo2.writeMicroseconds(servo2PosFiltered);
            
        }     // end of timed loop         

   
}       // end  of main loop
