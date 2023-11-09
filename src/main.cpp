#include <Arduino.h>
#include <SoftwareSerial.h>
#include <EncButton.h>
#include <GyverOLED.h>

#define TRANS_Rx_PIN 2
#define TRANS_Tx_PIN 3

#define PIN_BRIDGE_ENABLED 4 //enable bridge control
#define PIN_SPREADER_ENABLED 5 //enable spreader control
#define PIN_MINICRANES_ENABLED 6 //eneble minicranes control
#define PIN_MINICRANES_SYNCMODE 7 //minicrane twin mode
#define PIN_MINICRANES_ACTIVECRANENUM 8 //minicrane active crane
#define PIN_LIFTING_MECH_ENABLED 9 //minicrane twin mode
#define PIN_TWISTLOCK_STATE 10 //lock unlock twistlocks

#define GIMBAL_R_X_AXIS_PIN 20 //A6
#define GIMBAL_R_Y_AXIS_PIN 21 //A7
//#define GIMBAL1_BTN 12

#define GIMBAL_L_X_AXIS_PIN 18 //A4
#define GIMBAL_L_Y_AXIS_PIN 19 //A5
//#define GIMBAL2_BTN 13


#define TERMINATOR ';'

EncButton encoderServo(11, 12, 10);

SoftwareSerial Trans(TRANS_Rx_PIN,TRANS_Tx_PIN);

GyverOLED<SSD1306_128x64> oled;

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
  //pinMode(GIMBAL1_BTN, INPUT);   //but1

  pinMode(GIMBAL_L_X_AXIS_PIN, INPUT);//hor2
  pinMode(GIMBAL_L_Y_AXIS_PIN, INPUT);//ver2 
  //pinMode(GIMBAL2_BTN, INPUT);   //but2

  pinMode(PIN_BRIDGE_ENABLED, INPUT_PULLUP);

  pinMode(PIN_SPREADER_ENABLED, INPUT_PULLUP);
  pinMode(PIN_TWISTLOCK_STATE, INPUT_PULLUP);

  pinMode(PIN_MINICRANES_ENABLED, INPUT_PULLUP);
  pinMode(PIN_MINICRANES_SYNCMODE, INPUT_PULLUP);

  oled.init();
  oled.clear();

  oled.print                        // Вывод всех пунктов
  (F(
     " Servo 1  0  180\r\n"   // Не забываем про '\r\n' - символ переноса строки
     " Servo 2  0  180\r\n"
     " Servo 3  0  180\r\n"
     " Servo 4  0  180\r\n"
     " Servo 5  0  180\r\n"
     " Servo 6  0  180\r\n"
     " Srv 1-6  0  180\r\n"
     " Remote\r\n"
   ));

  oled.setCursor(0, pointer);
  oled.print('>');

  oled.update();  
}

uint32_t tmr = millis();

void loop()
{
  encoderServo.tick();
  if (encoderServo.turn())
  {
    switch (flag)
      {
        case 1:
          if(encoderServo.right()) { data_bot[pointer]++; data_bot[pointer] = constrain(data_bot[pointer], 0, 90);}
          if(encoderServo.left()) { data_bot[pointer]--; data_bot[pointer] = constrain(data_bot[pointer], 0, 90);}
        break;
        case 2:
          if(encoderServo.right()) { data_top[pointer]++; data_top[pointer] = constrain(data_top[pointer], 90, 180);}
          if(encoderServo.left()) { data_top[pointer]--; data_top[pointer] = constrain(data_top[pointer], 90, 180);}
        break;
        default:
        
          if(encoderServo.right()) pointer++;
          if(encoderServo.left()) pointer--;
          
          if(pointer > 7) pointer = 0;
          if(pointer == 255) pointer = 7;
          Serial.println(pointer);
        break;
      }

    pointer = constrain(pointer, 0, 7);
         
    for (int i = 0; i < 8; i++)
    {
      oled.setCursor(param_pos[0]*6, i);
      oled.print(' ');

      oled.setCursor(param_pos[1]*6, i);
      oled.print(' ');
      
      oled.setCursor(param_pos[2]*6, i);
      oled.print(' ');
    }
    if(pointer < 7)
    {
    oled.setCursor((param_pos[1] + 1)*6, pointer);
    oled.print(data_bot[pointer]);
    if(data_bot[pointer] < 10)
    {
      oled.setCursor((param_pos[1] + 2)*6, pointer);
      oled.print(' ');
    }
    

    oled.setCursor((param_pos[2] + 1)*6, pointer);
    oled.print(data_top[pointer]);
    }

    oled.setCursor(param_pos[flag]*6, pointer);
    oled.print('>');
    oled.update();
    //Serial.println(pointer);
  }
  
  if (encoderServo.click())
  {    
    if(pointer == 7)
    {
      remote = !remote;
      oled.setCursor(6*2, 7);
      oled.print(remote ? "Remote" : "Manual");
      oled.update();
      //Serial.println(remote ? "Remote" : "Manual");
    }
    else// if(pointer > 0)
    {
      if(flag == 2)
        flag = 0;
      else
        flag++;
      
      //flag = constrain(flag, 0, 2);
      /*
      for (int i = 1; i < 8; i++)
      {
        if(i != pointer)
        {
          oled.setCursor(param_pos[flag], i);
          oled.print(' ');
        }
      }
      */
      oled.setCursor(param_pos[flag == 0 ? 2 : flag - 1]*6, pointer);
      oled.print(' ');
      oled.setCursor(param_pos[flag]*6, pointer);
      oled.print('>');
      oled.update();
    }
  }
/*
  Servo 1     0    180
> Servo 2   > 0  > 180
  Servo 3     0    180
  Servo 4     0    180
  Servo 5     0    180
  Servo 6     0    180
  Srv 1-6     0    180
  Remote
*/
  if (encoderServo.hold())
  {
    oled.setCursor(75, pointer);
    oled.print(' ');
    oled.update();
  }

if(millis() - tmr > 50)
{
  tmr = millis();
  //Serial.println(roundf(map(analogRead(A0), 75, 592, 0, 180)/5)*5);
}

  int GIMBAL_R_X = analogRead(GIMBAL_R_X_AXIS_PIN);
  int GIMBAL_R_Y = analogRead(GIMBAL_R_Y_AXIS_PIN);
  //int GIMBAL_L_X = analogRead(GIMBAL_L_X_AXIS_PIN);
  int GIMBAL_L_Y = analogRead(GIMBAL_L_Y_AXIS_PIN);
  int TWISTLOCKS_STATE = digitalRead(PIN_MINICRANES_ENABLED) * 180;
  int MINICRANE_SYNC_STATE = digitalRead(PIN_MINICRANES_SYNCMODE);
  int MINICRANE_ACTIVE_CRANE = digitalRead(PIN_MINICRANES_ACTIVECRANENUM);
  int LIFT_UP_DOWN_VAL = map(GIMBAL_R_Y, 0, 1023, 0, 180);


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
  }
}