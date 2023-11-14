#include <Arduino.h>
#include <SoftwareSerial.h>
#include <EncButton.h>
#include <GyverOLED.h>

//////////////////////////////////////////////////
///////////  HC-12 transmitter pins  /////////////
//////////////////////////////////////////////////
#define TRANS_Rx_PIN 2
#define TRANS_Tx_PIN 3
SoftwareSerial Trans(TRANS_Rx_PIN,TRANS_Tx_PIN);

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

//////////////////////////////////////////////////
///////////  Terminator              /////////////
//////////////////////////////////////////////////
#define TERMINATOR ';'

//////////////////////////////////////////////////
///////////  IIC OLED                /////////////
//////////////////////////////////////////////////
GyverOLED<SSD1306_128x64> oled;

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

void setup()
{
  Trans.begin(115200);
  Serial.begin(115200);

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

  oled.print(F("cookies"));

  oled.setCursor(0, pointer);
  oled.print('>');

  oled.update();
  oled.setPower(OLED_DISPLAY_OFF);
}



void loop()
{
  if(millis() - tmr > 200)
  {
    tmr = millis();

    //Serial.println();
  }
  encoderButton.tick();
  if (encoderButton.turn())
  {    
    oled.setCursor(param_pos[flag]*6, pointer);
    oled.print('>');
    oled.update();
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
  int LIFT_UP_DOWN_VAL = round(map(GIMBAL_R_Y, 100, 808, 0, 180)/5)*5;


  //мост включен
  if(digitalRead(PIN_BRIDGE_ENABLED) == LOW)
  {      
    Trans.write("0,");
    Trans.write(map(GIMBAL_R_X, 0, 1023, -255, 255));//PWM //drive bridge
    Trans.write(',');
    Trans.write(map(GIMBAL_R_Y, 0, 1023, -255, 255));//PWM //drive trolley
    Trans.write(',');
    Trans.write(map(GIMBAL_L_Y, 0, 1023, -255, 255));//PWM //drive winch
    Trans.write(TERMINATOR);
  }

  //либо спредер включен
  else if(digitalRead(PIN_SPREADER_ENABLED) == LOW)
  {
    Trans.write("1,");
    Trans.write(map(GIMBAL_R_X, 0, 1023, -255, 255));//PWM //rotate
    Trans.write(',');
    Trans.write(map(GIMBAL_L_Y, 0, 1023, -255, 255));//PWM //drive telescopes
    Trans.write(',');
    Trans.write(TWISTLOCKS_STATE);//ANGLE //lock unlock twistlocks
    Trans.write(TERMINATOR);
  }

  //либо миникраны включены
  else if(digitalRead(PIN_MINICRANES_ENABLED) == LOW)
  {
    Trans.write("2,");
    Trans.write(MINICRANE_SYNC_STATE);//INT //work syncrously    
    Trans.write(',');
    Trans.write(MINICRANE_ACTIVE_CRANE);//INT //working active crane  
    Trans.write(',');
    Trans.write(map(GIMBAL_R_X, 0, 1023, -255, 255));//PWM //Rotate
    Trans.write(',');
    Trans.write(map(GIMBAL_L_Y, 0, 1023, -255, 255));//PWM //drive winch
    Trans.write(',');
    Trans.write(map(GIMBAL_R_Y, 0, 1023, -255, 255));//PWM //arm up down
    Trans.write(TERMINATOR);
  }

  //либо подъемники включены
  else if(digitalRead(PIN_LIFTING_MECH_ENABLED) == LOW)
  {    
    Trans.write("3,");
    Trans.write(LIFT_UP_DOWN_VAL);
    Trans.write(',');
    Trans.write(pointer);
    Trans.write(TERMINATOR);

    Serial.print("3,");
    Serial.print(LIFT_UP_DOWN_VAL);
    Serial.print(',');
    Serial.print(pointer);
    Serial.println(TERMINATOR);
  }
}