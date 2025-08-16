#include "WProgram.h"
#include "LegoPfReceiver1.h"

uint32_t LegoPfReceiver1::elapsedSince(uint32_t since, uint32_t now)
{
  return (since < now) ? now-since : 0;
}

uint32_t LegoPfReceiver1::elapsedSince(uint32_t since)
{
  return elapsedSince(since, micros());
}

void LegoPfReceiver1::setIRpin(uint8_t IR)
{
  PIN_IR = IR;
  led_count = 0;
}

void LegoPfReceiver1::setIRled(uint8_t LED)
{
  PIN_LED = LED;
}

uint32_t LegoPfReceiver1::irRecvSignal()
{
  if (led_count == 2)
  {
	digitalWrite(PIN_LED, HIGH); 
    led_count = 0;
  } 
  while (digitalRead(PIN_IR) == HIGH) {};
  digitalWrite(PIN_LED, LOW);
  led_count++;
  while (digitalRead(PIN_IR) == LOW) {};
  
  return elapsedSince(isrLastTimeStamp);
}

boolean LegoPfReceiver1::irRecv()
{
  uint32_t time = 0;
  while (!IS_S(time)) // zolang geen startbit ga door.
  {
	isrLastTimeStamp = micros();
    time = irRecvSignal(); 
  }
  uint16_t bits = 0;
  // startbit gevonden, verwacht nu 16 bits code.
  for(int i = 0; i < 16; i++ )
  {
	isrLastTimeStamp = micros();
    bits <<= 1;
    time = irRecvSignal();
    if( IS_1(time) ) //is bit een 1
    {
      bits |= 1;
    }
    else if( IS_0(time) ) // is bit een 0
    {
      //bits |= 0; (bitlengte word elke lus met 1 verlengt met 0 (bits <<= 1)
    }
    else
    {
      return false; //iets fout gegegaan
    }
  }
  uint8_t LRC;
  // maak er nibbles van en doe fout check.
  Nibble1 = ((unsigned int)bits >> 12) & 15;
  Nibble2 = ((unsigned int)bits >> 8) & 15;
  Nibble3 = ((unsigned int)bits >> 4) & 15;
  LRC = ((unsigned int)bits) & 15;
  if ((0xf ^ Nibble1 ^ Nibble2 ^ Nibble3) == LRC)
  {
    return true;
  } 
  return false;
}

LegoPfReceiver1 LEGOPFRECEIVER1;