#include <SoftwareSerial.h>
#include "timer_handler.h"
#include"buzzer_handler.h"
#include "ultrasonic_handler.h"
#define DEBUG true
  #if !DEBUG
  #define log(...)
  #else
  #define log(fmt, ...) consoleLog(fmt, ##__VA_ARGS__)
#endif


// CONSTS
#define MAXSPEED 433
#define STEP 24

  // Only use port 0 to 7;
#define dirLeft 3
#define stepLeft 2
#define dirRight 5
#define stepRight 4

#define TXpin 7
#define RXpin 8
#define STATEpin 12

#define TriggerPin 9
#define EchoPin 10

#define BUZZER_PIN 11
//------------------------------------------------
#define dirRightBackward 0b00000000 | 1 << dirRight     // 00100000
#define dirRightForward (0b11111111 & ~(1 << dirRight)) // 11011111
#define stepRightHIGH 0b00000000 | 1 << stepRight       // 00010000
#define stepRightLOW (0b11111111 & ~(1 << stepRight))   // 11101111
#define dirLeftForward 0b00000000 | 1 << dirLeft        // 00001000
#define dirLeftBackward (0b11111111 & ~(1 << dirLeft))  // 11110111
#define stepLeftHIGH 0b00000000 | 1 << stepLeft         // 00000100
#define stepLeftLOW (0b11111111 & ~(1 << stepLeft))     // 11111011
#define GYRO_ADDRESS 0x68
//------------------------------------------------
SoftwareSerial HC05(TXpin, RXpin);   // TX | RX of hc05
//------------------------------------------------


char received_byte;

int  throttle_left_motor, throttle_counter_left_motor, throttle_left_motor_memory;
int  throttle_right_motor, throttle_counter_right_motor, throttle_right_motor_memory;

int receive_counter;
int gyro_pitch_data_raw, gyro_yaw_data_raw, accelerometer_data_raw;


float speed;
float turn;
bool stop = false;

bool want_send_distance = true;

unsigned long previousTimeBuzzer; 
unsigned long currentTimeBuzzer;
unsigned long previousTimeUltrasonic; 
unsigned long currentTimeUltrasonic;

bool hc05_is_connected = false;
bool last_hc05_is_connected = !hc05_is_connected;
bool want_to_use_state = true;


void recieveData(){
  if(HC05.available()){                                                   
    received_byte = HC05.read();
    //log("recieved : ", received_byte);
    switch(received_byte){
      case 'w':
        turn = 0;
        stop=false;
        break;
      case 'd':
        if(turn + STEP/2 + speed < MAXSPEED){
        stop=false;
          turn-=STEP/2;
        }
        break;
      case 'a':
        if(turn + STEP/2 + speed < MAXSPEED){
        stop=false;
          turn += STEP/2;
        }
        break;
      case 'o':
        if(speed-STEP-turn>=-MAXSPEED){
        stop=false;
          speed-=STEP;
        }
        break;
      case 'i':
        if(speed+STEP+turn<=MAXSPEED){
          stop=false;
          speed+=STEP;
        }
        break;
      case ' ':
        //speed = 0;
        stop = true;
        break;
      case 'q':
        //speed = 0;
        turn = -turn;
        break;
      case 'b':
        digitalWrite(BUZZER_PIN,!digitalRead(BUZZER_PIN));
        log("buzzer: ", digitalRead(BUZZER_PIN));
        break;
      case 'm':
        want_send_distance = !want_send_distance;
        break;
      case 't':
        want_to_use_state = !want_to_use_state;
        buzzer::beep(1, 70);
        break;
    }
  }
}
int convertToLinear(float to_convert){
  to_convert = (to_convert > 0) ? 405 - (1/(to_convert + 9)) * 5500 : ((to_convert < 0)?-405 - (1/(to_convert - 9)) * 5500:to_convert);
  //Calculate the needed pulse time for the left and right stepper motor controllers
  return (to_convert > 0) ?  400 - to_convert : ((to_convert < 0) ? -400 - to_convert : 0);
}
void setup(){
  Serial.begin(115200);

  HC05.begin(9600);
  log("~ Setup Started ~");
  timer::setup();
  ultraSonic::setup(TriggerPin, EchoPin);
  buzzer::setup(BUZZER_PIN);
  pinMode(stepLeft, OUTPUT);
  pinMode(dirLeft, OUTPUT);
  pinMode(stepRight, OUTPUT);
  pinMode(dirRight, OUTPUT);
  pinMode(13, OUTPUT);
  log("~ Setup Done ~");
  buzzer::beep(2, 40);
}
void handleSendDistance(){
  if(want_send_distance){
    currentTimeUltrasonic = millis();
    if (currentTimeUltrasonic - previousTimeUltrasonic >= 500){
      previousTimeUltrasonic = currentTimeUltrasonic;
      float distanceRead =ultraSonic::calculateDistance();
      HC05.println(distanceRead);
      log("distance: ", distanceRead);
    }
  }
}
void handleSlowStop(){

  if(stop){
    currentTimeBuzzer = millis();
    if (currentTimeBuzzer - previousTimeBuzzer >= 20){
      
    previousTimeBuzzer = currentTimeBuzzer;
    currentTimeBuzzer =millis();
    turn = 0;
    if(speed > 0){
      speed-=STEP/2;
    }
    else if (speed<0){
      speed+=STEP/2;
    }
    }
  }
}
void handleHC05Connetion(){
  hc05_is_connected = digitalRead(STATEpin);
  if(hc05_is_connected != last_hc05_is_connected){
    last_hc05_is_connected = hc05_is_connected;
    if(hc05_is_connected){
      buzzer::beep(2, 50);
    }
    else if(!hc05_is_connected){
      buzzer::beep(4, 500);
      stop = true;
    }
  }

}

void loop(){

  buzzer::HandleBuzzer();
  handleSendDistance();
  if(want_to_use_state){
    handleHC05Connetion();
  }
  recieveData();
  handleSlowStop();
  throttle_left_motor = convertToLinear(speed + turn);
  throttle_right_motor = convertToLinear(speed - turn);
  
  //log("throttle_left_motor:",throttle_left_motor,",throttle_right_motor:",throttle_right_motor,",speed:",speed,",turn:",turn);
}

