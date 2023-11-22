#include <Arduino.h>
#include <SoftwareSerial.h>
#include <EncButton.h>
#include <GyverOLED.h>

//////////////////////////////////////////////////
///////////  HC-12 transmitter pins  /////////////
//////////////////////////////////////////////////
#define TRANS_Rx_PIN 2
#define TRANS_Tx_PIN 3
#define SOFT_SERIAL_SPEED 38400

SoftwareSerial TransmitterSerial(TRANS_Rx_PIN,TRANS_Tx_PIN);

//////////////////////////////////////////////////
///////////  Switches pins           /////////////
//////////////////////////////////////////////////
#define PIN_BRIDGE_ENABLED 4 //enable bridge control
#define PIN_SPREADER_ENABLED 5 //enable spreader control
#define PIN_MINICRANES_ENABLED 6 //eneble minicranes control
#define PIN_MINICRANES_SYNCMODE 7 //minicrane twin mode
#define PIN_MINICRANES_ACTIVECRANENUM 8 //minicrane active crane
#define PIN_LIFTING_MECH_ENABLED 9 //minicrane twin mode
#define PIN_TWISTLOCK_STATE 10 //lock unlock twistlocks


//////////////////////////////////////////////////
///////////  Gimbal pins             /////////////
//////////////////////////////////////////////////
#define GIMBAL_R_X_AXIS_PIN A2 //A2
#define GIMBAL_R_Y_AXIS_PIN A3 //A3

#define GIMBAL_L_X_AXIS_PIN A6 //A6
#define GIMBAL_L_Y_AXIS_PIN A7 //A7

//////////////////////////////////////////////////
///////////  Encoder pins            /////////////
//////////////////////////////////////////////////
#define ENCODER_DIR1_PIN 11
#define ENCODER_DIR2_PIN 12
#define ENCODER_KEY_PIN 13
EncButton encoderButton(ENCODER_DIR1_PIN, ENCODER_DIR2_PIN, ENCODER_KEY_PIN);

//////////////////////////////////////////////////
///////////  Timers                  /////////////
//////////////////////////////////////////////////
uint32_t tmr = millis();
uint32_t tmr2 = millis();

//////////////////////////////////////////////////
///////////  Terminator              /////////////
//////////////////////////////////////////////////
#define TERMINATOR ';'

//////////////////////////////////////////////////
///////////  IIC OLED                /////////////
//////////////////////////////////////////////////
GyverOLED<SSH1106_128x64> oled;

//////////////////////////////////////////////////
/////////// Variables                /////////////
//////////////////////////////////////////////////
int servoNumManual = 6;
byte prevPointer = 0;
byte pointer = 0;
bool remote = true;
int data_bot[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int data_top[8] = {180, 180, 180, 180, 180, 180, 180, 0};
int flag = 0;
int param_pos[3] = {0, 9, 12};
int GIMBAL_R_BOTTOM_VALUE = 100;
int GIMBAL_R_TOP_VALUE = 808;
int GIMBAL_R_ZERO_VALUE = 450;

void setup()
{
  pinMode(TRANS_Rx_PIN, INPUT);
  pinMode(TRANS_Tx_PIN, OUTPUT);

  TransmitterSerial.begin(SOFT_SERIAL_SPEED);
  Serial.begin(9600);

  pinMode(GIMBAL_R_X_AXIS_PIN, INPUT);//hor1
  pinMode(GIMBAL_R_Y_AXIS_PIN, INPUT);//ver1  

  pinMode(GIMBAL_L_X_AXIS_PIN, INPUT);//hor2
  pinMode(GIMBAL_L_Y_AXIS_PIN, INPUT);//ver2 

  pinMode(PIN_BRIDGE_ENABLED, INPUT_PULLUP);

  pinMode(PIN_SPREADER_ENABLED, INPUT_PULLUP);
  pinMode(PIN_TWISTLOCK_STATE, INPUT_PULLUP);

  pinMode(PIN_MINICRANES_ENABLED, INPUT_PULLUP);
  pinMode(PIN_MINICRANES_SYNCMODE, INPUT_PULLUP);

  pinMode(PIN_LIFTING_MECH_ENABLED, INPUT_PULLUP);

  oled.init();
  oled.clear();
  oled.home();
  oled.print("starting...");
  oled.update();
  //oled.setPower(OLED_DISPLAY_OFF);
}



void loop()
{
  encoderButton.tick();
  if (encoderButton.turn())
  {    
    //oled.setCursor(param_pos[flag]*6, pointer);
    //oled.print('>');
    //oled.update();
  }
  
  if (encoderButton.click())
  {    
    if(pointer == 7)
    {

    }
    else// if(pointer > 0)
    {
      
    }
  }

  if (encoderButton.hold())
  {

  }

//Serial.println("123");
/*
if(millis() - tmr > 200)
{
  tmr = millis();

  if(digitalRead(PIN_BRIDGE_ENABLED) == LOW) Serial.println("Bridge Enabled");
  if(digitalRead(PIN_SPREADER_ENABLED) == LOW) Serial.println("Spreader Enabled");
  if(digitalRead(PIN_LIFTING_MECH_ENABLED) == LOW) Serial.println("Lifting Enabled");
  if(digitalRead(PIN_MINICRANES_ENABLED) == LOW) Serial.println("Mini Cranes Enabled");
}
*/
//if(millis() - tmr > 50)
//{
  //tmr = millis();
  //Serial.println(roundf(map(analogRead(A0), 75, 592, 0, 180)/5)*5);
//}

  int GIMBAL_R_X = analogRead(GIMBAL_R_X_AXIS_PIN);
  int GIMBAL_R_Y = analogRead(GIMBAL_R_Y_AXIS_PIN);
  //int GIMBAL_L_X = analogRead(GIMBAL_L_X_AXIS_PIN);
  int GIMBAL_L_Y = analogRead(GIMBAL_L_Y_AXIS_PIN);
  int TWISTLOCKS_STATE = digitalRead(PIN_MINICRANES_ENABLED) * 180;
  int MINICRANE_SYNC_STATE = digitalRead(PIN_MINICRANES_SYNCMODE);
  int MINICRANE_ACTIVE_CRANE = digitalRead(PIN_MINICRANES_ACTIVECRANENUM);
  int LIFT_UP_DOWN_VAL = map(GIMBAL_R_Y, 100, 808, 0, 180);


  //мост включен
  if(digitalRead(PIN_BRIDGE_ENABLED) == LOW)
  {      
    TransmitterSerial.write("0,");
    TransmitterSerial.write(map(GIMBAL_R_X, 0, 1023, -255, 255));//PWM //drive bridge
    TransmitterSerial.write(',');
    TransmitterSerial.write(map(GIMBAL_R_Y, 0, 1023, -255, 255));//PWM //drive trolley
    TransmitterSerial.write(',');
    TransmitterSerial.write(map(GIMBAL_L_Y, 0, 1023, -255, 255));//PWM //drive winch
    TransmitterSerial.write(TERMINATOR);
  }

  //либо спредер включен
  else if(digitalRead(PIN_SPREADER_ENABLED) == LOW)
  {
    TransmitterSerial.write("1,");
    TransmitterSerial.write(map(GIMBAL_R_X, 0, 1023, -255, 255));//PWM //rotate
    TransmitterSerial.write(',');
    TransmitterSerial.write(map(GIMBAL_L_Y, 0, 1023, -255, 255));//PWM //drive telescopes
    TransmitterSerial.write(',');
    TransmitterSerial.write(TWISTLOCKS_STATE);//ANGLE //lock unlock twistlocks
    TransmitterSerial.write(TERMINATOR);
  }

  //либо миникраны включены
  else if(digitalRead(PIN_MINICRANES_ENABLED) == LOW)
  {
    TransmitterSerial.write("2,");
    TransmitterSerial.write(MINICRANE_SYNC_STATE);//INT //work syncrously    
    TransmitterSerial.write(',');
    TransmitterSerial.write(MINICRANE_ACTIVE_CRANE);//INT //working active crane  
    TransmitterSerial.write(',');
    TransmitterSerial.write(map(GIMBAL_R_X, 0, 1023, -255, 255));//PWM //Rotate
    TransmitterSerial.write(',');
    TransmitterSerial.write(map(GIMBAL_L_Y, 0, 1023, -255, 255));//PWM //drive winch
    TransmitterSerial.write(',');
    TransmitterSerial.write(map(GIMBAL_R_Y, 0, 1023, -255, 255));//PWM //arm up down
    TransmitterSerial.write(TERMINATOR);
  }

  //либо подъемники включены
  else if(digitalRead(PIN_LIFTING_MECH_ENABLED) == LOW)
  {    
    TransmitterSerial.write('3');
    TransmitterSerial.write(',');
    TransmitterSerial.write(LIFT_UP_DOWN_VAL);
    TransmitterSerial.write(',');
    TransmitterSerial.write(6);
    TransmitterSerial.write(TERMINATOR);

    Serial.print("Lifting...   ");
    Serial.print(3);
    Serial.print(',');
    Serial.print(LIFT_UP_DOWN_VAL);
    Serial.print(',');
    Serial.print(6);
    Serial.println(TERMINATOR);

    //oled.clear();
    //oled.home();
    //oled.print('1');
    //oled.update();
    }
  }
