#include <Arduino.h>
#include <SoftwareSerial.h>
#include <EncButton.h>
#include <GyverOLED.h>
#include <RF24.h>

//////////////////////////////////////////////////
///////////        DATA PACKAGE      /////////////
//////////////////////////////////////////////////
struct Data_Package
{
  byte Bridge_Enabled;
  byte Bridge_Drive;
  byte Bridge_Trolley;
  byte Bridge_Winch;

  byte Spreader_Enabled;
  byte Spreader_Rotate;
  byte Spreader_Telescopes;
  byte Spreader_Twistlocks;

  byte Mini_Enabled;
  byte Mini_Mode;
  byte Mini_ActiveCrane;
  byte Mini_Rotate;
  byte Mini_Winch;
  byte Mini_Arm;

  byte Lift_Enabled;
  byte Lift_Drive;
  byte Lift_Servo;
};

Data_Package data;

//////////////////////////////////////////////////
///////////  Communication           /////////////
//////////////////////////////////////////////////
//#define TRANS_Rx_PIN 2
//#define TRANS_Tx_PIN 3
//#define SOFT_SERIAL_SPEED 38400
//SoftwareSerial TransmitterSerial(TRANS_Rx_PIN,TRANS_Tx_PIN);
RF24 radio(2, 3);// nRF24L01 (CE, CSN)
const byte address[6] = "00001"; // Address

//////////////////////////////////////////////////
///////////  Switches pins           /////////////
//////////////////////////////////////////////////
#define PIN_BRIDGE_ENABLED 4 //enable bridge control
#define PIN_SPREADER_ENABLED 5 //enable spreader control
#define PIN_MINICRANES_ENABLED 6 //eneble minicranes control
#define PIN_MINICRANES_MODE 7 //minicrane twin mode
#define PIN_MINICRANES_ACTIVECRANENUM 8 //minicrane active crane
#define PIN_LIFT_ENABLED 9 //minicrane twin mode
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
  //pinMode(TRANS_Rx_PIN, INPUT);
  //pinMode(TRANS_Tx_PIN, OUTPUT);

  //TransmitterSerial.begin(SOFT_SERIAL_SPEED);
  Serial.begin(9600);

  // Define the radio communication
  radio.begin();
  radio.openWritingPipe(address);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);

  //pinMode(GIMBAL_R_X_AXIS_PIN, INPUT);//hor1
  //pinMode(GIMBAL_R_Y_AXIS_PIN, INPUT);//ver1  

  //pinMode(GIMBAL_L_X_AXIS_PIN, INPUT);//hor2
  //pinMode(GIMBAL_L_Y_AXIS_PIN, INPUT);//ver2 

  pinMode(PIN_BRIDGE_ENABLED, INPUT_PULLUP);

  pinMode(PIN_SPREADER_ENABLED, INPUT_PULLUP);
  pinMode(PIN_TWISTLOCK_STATE, INPUT_PULLUP);

  pinMode(PIN_MINICRANES_ENABLED, INPUT_PULLUP);
  pinMode(PIN_MINICRANES_MODE, INPUT_PULLUP);

  pinMode(PIN_LIFT_ENABLED, INPUT_PULLUP);

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
  if(digitalRead(PIN_LIFT_ENABLED) == LOW) Serial.println("Lifting Enabled");
  if(digitalRead(PIN_MINICRANES_ENABLED) == LOW) Serial.println("Mini Cranes Enabled");
}
*/
//if(millis() - tmr > 50)
//{
  //tmr = millis();
  //Serial.println(roundf(map(analogRead(A0), 75, 592, 0, 180)/5)*5);
//}



  byte GIMBAL_R_X = map(analogRead(GIMBAL_R_X_AXIS_PIN), 0, 1023, 0, 255);
  byte GIMBAL_R_Y = map(analogRead(GIMBAL_R_Y_AXIS_PIN), 0, 1023, 0, 255);
  byte GIMBAL_L_X = map(analogRead(GIMBAL_L_X_AXIS_PIN), 0, 1023, 0, 255);
  byte GIMBAL_L_Y = map(analogRead(GIMBAL_L_Y_AXIS_PIN), 0, 1023, 0, 255);

  byte BRIDGE_EN    = digitalRead(PIN_BRIDGE_ENABLED);
  byte SPREADER_EN  = digitalRead(PIN_SPREADER_ENABLED);
  byte MINI_EN      = digitalRead(PIN_MINICRANES_ENABLED);
  byte LIFT_EN      = digitalRead(PIN_LIFT_ENABLED);

  byte TWISTLOCKS_STATE = digitalRead(PIN_TWISTLOCK_STATE) * 180;
  byte MINICRANE_SYNC_STATE = digitalRead(PIN_MINICRANES_MODE);
  byte MINICRANE_ACTIVE_CRANE = digitalRead(PIN_MINICRANES_ACTIVECRANENUM);
  byte LIFT_UP_DOWN_VAL = map(GIMBAL_R_Y, 100, 808, 0, 180);

  data.Bridge_Enabled = digitalRead(PIN_BRIDGE_ENABLED);
  data.Bridge_Drive   = GIMBAL_R_X;
  data.Bridge_Trolley = GIMBAL_R_Y;
  data.Bridge_Trolley = GIMBAL_L_Y;

  data.Spreader_Enabled = digitalRead(PIN_SPREADER_ENABLED);
  data.Spreader_Rotate = GIMBAL_R_X;
  data.Spreader_Telescopes = GIMBAL_L_Y;
  data.Spreader_Twistlocks = digitalRead(PIN_TWISTLOCK_STATE) * 180;

  data.Mini_Enabled     = digitalRead(PIN_MINICRANES_ENABLED);
  data.Mini_Mode        = digitalRead(PIN_MINICRANES_MODE);
  data.Mini_ActiveCrane = digitalRead(PIN_MINICRANES_ACTIVECRANENUM);
  data.Mini_Rotate      = GIMBAL_R_X;
  data.Mini_Winch       = GIMBAL_L_Y;
  data.Mini_Arm         = GIMBAL_R_Y;

  data.Lift_Enabled = digitalRead(PIN_LIFT_ENABLED);
  data.Lift_Drive   = GIMBAL_R_Y;
  data.Lift_Servo   = 6;

  //мост включен
  if(digitalRead(PIN_BRIDGE_ENABLED) == LOW)
  {
    /*
    data.Bridge_Drive = map(GIMBAL_R_X, 0, 1023, 0, 255);
    data.Bridge_Trolley = map(GIMBAL_R_Y, 0, 1023, 0, 255);
    data.Bridge_Trolley = map(GIMBAL_L_Y, 0, 1023, 0, 255);
    // Send the whole data from the structure to the receiver
    radio.write(&data, sizeof(Data_Package));
    
    TransmitterSerial.write("0,");
    TransmitterSerial.write(map(GIMBAL_R_X, 0, 1023, -255, 255));//PWM //drive bridge
    TransmitterSerial.write(',');
    TransmitterSerial.write(map(GIMBAL_R_Y, 0, 1023, -255, 255));//PWM //drive trolley
    TransmitterSerial.write(',');
    TransmitterSerial.write(map(GIMBAL_L_Y, 0, 1023, -255, 255));//PWM //drive winch
    TransmitterSerial.write(TERMINATOR);
    */
  }

  //либо спредер включен
  else if(digitalRead(PIN_SPREADER_ENABLED) == LOW)
  {
    /*
    data.Spreader_Rotate = map(GIMBAL_R_X, 0, 1023, 0, 255);
    data.Spreader_Telescopes = map(GIMBAL_L_Y, 0, 1023, 0, 255);
    data.Spreader_Twistlocks = digitalRead(spreader) * 180;
    
    TransmitterSerial.write("1,");
    TransmitterSerial.write(map(GIMBAL_R_X, 0, 1023, -255, 255)); // PWM //rotate
    TransmitterSerial.write(',');
    TransmitterSerial.write(map(GIMBAL_L_Y, 0, 1023, -255, 255)); // PWM //drive telescopes
    TransmitterSerial.write(',');
    TransmitterSerial.write(TWISTLOCKS_STATE); // ANGLE //lock unlock twistlocks
    TransmitterSerial.write(TERMINATOR);
    */
  }

  //либо миникраны включены
  else if(digitalRead(PIN_MINICRANES_ENABLED) == LOW)
  {
    /*
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
    */
  }

  //либо подъемники включены
  else if(digitalRead(PIN_LIFT_ENABLED) == LOW)
  {    
    /*
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
    */
    }
  }
