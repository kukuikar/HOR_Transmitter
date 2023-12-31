#include <Arduino.h>
//#include <SoftwareSerial.h>
//#include <EncButton.h>
//#include <GyverOLED.h>
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
#define TRANS_CE_PIN 9
#define TRANS_CSN_PIN 10
RF24 radio(TRANS_CE_PIN, TRANS_CSN_PIN);// nRF24L01 (CE, CSN)
const byte address[6] = "00001"; // Address

//////////////////////////////////////////////////
///////////  Switches pins           /////////////
//////////////////////////////////////////////////
#define PIN_BRIDGE_ENABLED            2 //enable bridge control
#define PIN_SPREADER_ENABLED          3 //enable spreader control
#define PIN_MINICRANES_ENABLED        4 //eneble minicranes control
#define PIN_MINICRANES_MODE           5 //minicrane twin mode
#define PIN_MINICRANES_ACTIVECRANENUM 6 //minicrane active crane
#define PIN_LIFT_ENABLED              7 //lift enabled
#define PIN_TWISTLOCK_STATE           8 //lock unlock twistlocks


//////////////////////////////////////////////////
///////////  Gimbal pins             /////////////
//////////////////////////////////////////////////
#define GIMBAL_R_X_AXIS_PIN A2 //A2
#define GIMBAL_R_Y_AXIS_PIN A3 //A3

#define GIMBAL_L_X_AXIS_PIN A6 //A6
#define GIMBAL_L_Y_AXIS_PIN A7 //A7

//////////////////////////////////////////////////
///////////  Encoder                 /////////////
//////////////////////////////////////////////////
//#define ENCODER_DIR1_PIN  11
//#define ENCODER_DIR2_PIN  12
//#define ENCODER_KEY_PIN   13
//EncButton encoderButton(ENCODER_DIR1_PIN, ENCODER_DIR2_PIN, ENCODER_KEY_PIN);

//////////////////////////////////////////////////
///////////  Timers                  /////////////
//////////////////////////////////////////////////
uint32_t tmr = millis();
uint32_t tmr2 = millis();

//////////////////////////////////////////////////
///////////  IIC OLED                /////////////
//////////////////////////////////////////////////
//GyverOLED<SSH1106_128x64> oled;

//////////////////////////////////////////////////
/////////// Variables                /////////////
//////////////////////////////////////////////////


void setup()
{
  Serial.begin(38400);

  //Define the radio communication
  radio.begin();
  radio.openWritingPipe(address);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(0x60);

  pinMode(PIN_BRIDGE_ENABLED, INPUT_PULLUP);

  pinMode(PIN_SPREADER_ENABLED, INPUT_PULLUP);
  pinMode(PIN_TWISTLOCK_STATE, INPUT_PULLUP);

  pinMode(PIN_MINICRANES_ENABLED, INPUT_PULLUP);
  pinMode(PIN_MINICRANES_MODE, INPUT_PULLUP);
  pinMode(PIN_MINICRANES_ACTIVECRANENUM, INPUT_PULLUP);

  pinMode(PIN_LIFT_ENABLED, INPUT_PULLUP);
}
/*
    fastDW(SH_LD_165, LOW);
    fastDW(SH_LD_165, HIGH);
    for (int i = 0; i < 8; i++)
    {
        if (digitalRead(QH_165))
            buf_165[i] = 1;
        else
            buf_165[i] = 0;
        fastDW(CLK_165, HIGH);
        fastDW(CLK_165, LOW);
    }
    */

void loop()
{
  
  

  byte GIMBAL_R_X = map(analogRead(GIMBAL_R_X_AXIS_PIN), 0, 1023, 0, 255);

  byte GIMBAL_R_Y = map(analogRead(GIMBAL_R_Y_AXIS_PIN), 0, 1023, 0, 255);
  
  byte GIMBAL_L_X = map(analogRead(GIMBAL_L_X_AXIS_PIN), 0, 1023, 0, 255);
  byte GIMBAL_L_Y = map(analogRead(GIMBAL_L_Y_AXIS_PIN), 0, 1023, 0, 255);

  //Добавил, чтобы калибровать "большой" джойстик (всегда новые граници были при подключении)
  Serial.print("GIMBAL_R_Y_AXIS_PIN = ");
  Serial.println(analogRead(GIMBAL_R_Y_AXIS_PIN));
  Serial.print("GIMBAL_R_Y = ");
  Serial.println(GIMBAL_R_Y);
  Serial.print("Send:  ");
  Serial.println(data.Lift_Drive);
  delay(20);

  String S;
  S.reserve(100);
  char str[50];
  S.toCharArray(str,S.length()+1);
  //S.remove;
  /*

  
  byte BRIDGE_EN    = digitalRead(PIN_BRIDGE_ENABLED);
  byte SPREADER_EN  = digitalRead(PIN_SPREADER_ENABLED);
  byte MINI_EN      = digitalRead(PIN_MINICRANES_ENABLED);
  byte LIFT_EN      = digitalRead(PIN_LIFT_ENABLED);

  byte TWISTLOCKS_STATE       = digitalRead(PIN_TWISTLOCK_STATE) * 180;
  byte MINICRANE_SYNC_STATE   = digitalRead(PIN_MINICRANES_MODE);
  byte MINICRANE_ACTIVE_CRANE = digitalRead(PIN_MINICRANES_ACTIVECRANENUM);
  byte LIFT_UP_DOWN_VAL       = map(GIMBAL_R_Y, 100, 808, 0, 180);
  */

  // BRIDGE
  data.Bridge_Enabled = digitalRead(PIN_BRIDGE_ENABLED);
  data.Bridge_Drive   = GIMBAL_R_X;
  data.Bridge_Trolley = GIMBAL_R_Y;
  data.Bridge_Trolley = GIMBAL_L_Y;

  // SPREADER
  data.Spreader_Enabled     = digitalRead(PIN_SPREADER_ENABLED);
  data.Spreader_Rotate      = GIMBAL_R_X;
  data.Spreader_Telescopes  = GIMBAL_L_Y;
  data.Spreader_Twistlocks  = digitalRead(PIN_TWISTLOCK_STATE) * 180;

  // MINI CRANES
  data.Mini_Enabled     = digitalRead(PIN_MINICRANES_ENABLED);
  data.Mini_Mode        = digitalRead(PIN_MINICRANES_MODE);
  data.Mini_ActiveCrane = digitalRead(PIN_MINICRANES_ACTIVECRANENUM);
  data.Mini_Rotate      = GIMBAL_R_X;
  data.Mini_Winch       = GIMBAL_L_Y;
  data.Mini_Arm         = GIMBAL_R_Y;

  // LIFT
  data.Lift_Enabled = digitalRead(PIN_LIFT_ENABLED);
  data.Lift_Drive   = map(GIMBAL_R_Y, 0, 255, 0, 180);
  data.Lift_Servo   = 6;

  radio.write(&data, sizeof(Data_Package));
  
  }