#include <tinyServo.h>
#include <LegoPfReceiver1.h>
#include <EEPROM.h>

template <class T> int EEPROM_write(int ee, const T& value)
{
    const byte* p = (const byte*)(const void*)&value;
    int i;
    for (i = 0; i < sizeof(value); i++)
	  EEPROM.write(ee++, *p++);
    return i;
}

template <class T> int EEPROM_read(int ee, T& value)
{
    byte* p = (byte*)(void*)&value;
    int i;
    for (i = 0; i < sizeof(value); i++)
	  *p++ = EEPROM.read(ee++);
    return i;
}

tinyServo servo1;
tinyServo servo2;

struct servopos
{
  byte Left;
  byte Center; 
  byte Right;
};

//Pin layout
#define DIP1 3
#define DIP2 1
#define DIP3 0
#define POTMETER 4
#define BUTTON 8
#define IRLED 9
#define IR_PIN 2

boolean Calibrate_Servo = false;

int PowerLed = 10000;
uint8_t buttonState = HIGH;
uint8_t lastButtonState = HIGH;
struct servopos conf1;
struct servopos conf2;

enum
{
  CENTER,
  LEFT,
  RIGHT,
} 
servoState = CENTER;

void setup()
{
  EEPROM_read(0, conf1);
  EEPROM_read(10, conf2);
  pinMode(IRLED, OUTPUT);
  digitalWrite(IRLED, HIGH);
  pinMode(BUTTON, INPUT);
  digitalWrite(BUTTON, HIGH);
  delay(100); //sleep for button
  while (digitalRead(BUTTON) == LOW)
  {
    Calibrate_Servo = true;
  }
  pinMode(DIP1, INPUT); // DIP1
  pinMode(DIP2, INPUT); // DIP2
  pinMode(DIP3, INPUT); // DIP3
  digitalWrite(DIP1, HIGH); //internal pullup
  digitalWrite(DIP2, HIGH); //internal pullup
  digitalWrite(DIP3, HIGH); //internal pullup
  
  servo1.attach(5);
  servo2.attach(4);
  servo1.write((byte)conf1.Center); // center
  servo2.write((byte)conf2.Center); // center
  
  LEGOPFRECEIVER1.setIRpin(IR_PIN);
  LEGOPFRECEIVER1.setIRled(IRLED);  
}

void loop()
{
  if (!CalibrateServo())
  {
    Handle_PF_Commands();
  }
}

boolean CalibrateServo()
{
  if (Calibrate_Servo)
  {
    uint16_t PotVal;
    uint8_t SelCH = Selected_Channel(); 
    
    PotVal = analogRead(POTMETER);
    PotVal = map(PotVal, 0, 1023, 0, 180);  
    
    int reading = digitalRead(BUTTON);    
    if (reading != lastButtonState) {
      buttonState = reading;
    } 
    lastButtonState = reading;

    if (buttonState == LOW)
    {  
      if (SelCH == 1) 
      {
        caliServoState(PotVal, &conf1);
        if (Calibrate_Servo == false)
          EEPROM_write(0, conf1);
      } else if (SelCH == 2)
      {
        caliServoState(PotVal, &conf2);
        if (Calibrate_Servo == false)
          EEPROM_write(10, conf2);
      }
      while (digitalRead(BUTTON) == LOW);
      buttonState = HIGH;
    }
    if (SelCH == 1)
      servo1.write(PotVal);
    else if (SelCH == 2)
      servo2.write(PotVal);
    else
    {
      digitalWrite(IRLED, LOW);
      delay(150);
      digitalWrite(IRLED, HIGH);
      delay(150);
    }
  }
  return Calibrate_Servo;
}

void caliServoState(uint16_t PotVal, struct servopos *conf)
{
  switch (servoState)
  {
    case CENTER:
      (*conf).Center = PotVal;
      servoState = LEFT;
      digitalWrite(IRLED, HIGH);
      //delay(1000);
      break;
    case LEFT:
      (*conf).Left = PotVal;
      servoState = RIGHT;
      digitalWrite(IRLED, LOW);
      //delay(1000);
      break;
    case RIGHT:
      (*conf).Right = PotVal;
      servoState = CENTER;       
      Calibrate_Servo = false;
      delay(1000);
      break;
  }
}

uint8_t Selected_Channel()
{
  uint8_t channel = 0;
  if (digitalRead(DIP1) == LOW) // DIP1
  {
    channel |= B1;
  }
  if (digitalRead(DIP2) == LOW) // DIP2
  {
    channel |= B10;
  }
  if (digitalRead(DIP3) == LOW) // DIP3
  {
    channel |= B100;
  }
  return channel;
}

void Handle_PF_Commands()
{
  digitalWrite(IRLED, HIGH);
  if (LEGOPFRECEIVER1.irRecv())
  {
    uint8_t Channel, Escape, Address; 
    Escape = (unsigned int)LEGOPFRECEIVER1.Nibble1 >> 2 & 1;
    Channel = (unsigned int)LEGOPFRECEIVER1.Nibble1 & 3;  
    
    if (Escape == 0)
    {
      Address = (unsigned int)LEGOPFRECEIVER1.Nibble2 >> 3 & 1;
      uint8_t Toggle = (unsigned int)LEGOPFRECEIVER1.Nibble1 >> 3 & 1;    
      uint8_t mode = (unsigned int)LEGOPFRECEIVER1.Nibble2 & 7;   
      uint8_t Data = LEGOPFRECEIVER1.Nibble3;
      
      uint8_t A_output, B_output, Output, Pin, Function, Mode;

      if (Address == 1) //Extra address space
      {
        Channel |= B100;
      }

      if (Selected_Channel() == Channel)
      {
        //digitalWrite(IRLED, LOW);
        switch (mode)
        {
          case 1: //Combo direct (timeout)                  
            B_output = (unsigned int)Data >> 2 & 3;
            A_output = Data & 3; 
            Combo_Direct_mode(A_output, B_output);      
            break;    
          case 2: //Single pin continuous (no timeout) (Not Implemented)
            Output = Data >> 3 & 1;
            Pin = Data >> 2 & 1;
            Function = Data & 3;
          
            break;
          case 3: //Single pin timeout (Not Implemented)
            Output = Data >> 3 & 1;
            Pin = Data >> 2 & 1;
            Function = Data & 3;

            break;
          default: //Single output (Not Implemented)
            Mode = mode >> 2 & 1;
            Output = mode & 1;

            break;
        }
      }
    } else //Combo PWM Mode
    {      
      Address = (unsigned int)LEGOPFRECEIVER1.Nibble1 >> 3 & 1;
      uint8_t A_output, B_output;
      
      if (Address == 1) //Extra address space
      {
        Channel |= B100;
      }
         
      if (Selected_Channel() == Channel)
      {    
        //digitalWrite(IRLED, LOW);
        B_output = LEGOPFRECEIVER1.Nibble2;
        A_output = LEGOPFRECEIVER1.Nibble3;
        Combo_PWM_mode(A_output, B_output);
      }
    }
  }
}

void Combo_Direct_mode(uint8_t A_output, uint8_t B_output)
{
  directrun(A_output, servo1, conf1);
  directrun(B_output, servo2, conf2);
}

void directrun(uint8_t output, tinyServo servo, struct servopos conf)
{
  switch (output)
  {
    case 0: // float
      servo.write((byte)conf.Center);
      break;
    case 1: // forward      
      servo.write((byte)conf.Left);
      break;
    case 2: // backward
      servo.write((byte)conf.Right);
      break;
    case 3: // brake
      servo.write((byte)conf.Center);
      break;
  }
}

void Combo_PWM_mode(uint8_t A_output, uint8_t B_output)
{
  pwmrun(A_output, servo1, conf1);
  pwmrun(B_output, servo2, conf2);
}

void pwmrun(uint8_t output, tinyServo servo, struct servopos conf)
{
  if (output == 0) // float
  {
    servo.write((byte)conf.Center);
  } else if (output == 8) // break
  {
    servo.write((byte)conf.Center);
  } else
  {
    if (output < 8) //FORWARD
    {
      servo.write(map(output, 0, 7, (byte)conf.Center - 1, (byte)conf.Left));
    } else //BACKWARD
    {
      servo.write(map(output, 16, 9, (byte)conf.Center + 1, (byte)conf.Right));   
    }
  }
}
