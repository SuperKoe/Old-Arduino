#include "WProgram.h"
#include "LegoPfReceiver.h"

void SETirRecvSignal()
{
  LEGOPFRECEIVER.irRecvSignal();
}

uint32_t LegoPfReceiver::elapsedSince(uint32_t since, uint32_t now)
{
  return (since < now) ? now-since : 0;
}

uint32_t LegoPfReceiver::elapsedSince(uint32_t since)
{
  return elapsedSince(since, micros());
}

//Infrared varibels
enum
{
  ISR_IDLE,
  ISR_START,
  ISR_BIT_ON,
}
volatile isrState = ISR_IDLE;

volatile uint32_t isrLastTimeStamp;
volatile uint16_t isrRcvCmd;
volatile uint16_t isrNewCmd;
volatile uint8_t isrBitCnt;

void LegoPfReceiver::SetInterrupt(int INT) 
{
  attachInterrupt(INT, SETirRecvSignal, FALLING);
}

void LegoPfReceiver::irRecvSignal()
{
  volatile unsigned elapsed;
  {
    uint32_t timeStamp = micros();
    elapsed = elapsedSince(isrLastTimeStamp,timeStamp);
    isrLastTimeStamp = timeStamp;
  }
  switch(isrState)
  {
    case ISR_IDLE:
      isrState = ISR_START;
      
      break;
    case ISR_START:
      if (IS_S(elapsed))
      {
        isrBitCnt = 0;
        isrNewCmd = 0;
        isrState = ISR_BIT_ON;
        //Serial.println("Start");
      } else
      {
         isrState = ISR_IDLE;
         //delayMicroseconds(1200);
         //Serial.println("error");
      }   
      break;
    case ISR_BIT_ON:
      isrNewCmd <<= 1;
      isrBitCnt++;
      if (IS_1(elapsed))
      {
        isrNewCmd |= 1;
        //Serial.println("1");
      }
      else if (IS_0(elapsed))
      {
        // isrNewCmd |= 0;
        //Serial.println("0");
      } else
      {
        isrState = ISR_START;
        //delayMicroseconds(1200);
        break;
      }
      if (isrBitCnt == 16)
      {
        isrState = ISR_IDLE;
        isrRcvCmd = isrNewCmd;
      }
      break;
  }
}

boolean LegoPfReceiver::irRecv()
{
  uint16_t cmd = isrRcvCmd;
  isrRcvCmd = 0;
  uint8_t LRC;
  if (cmd)
  {
    Nibble1 = ((unsigned int)cmd >> 12) & 15;
    Nibble2 = ((unsigned int)cmd >> 8) & 15;
    Nibble3 = ((unsigned int)cmd >> 4) & 15;
    LRC = ((unsigned int)cmd) & 15;
    if ((0xf ^ Nibble1 ^ Nibble2 ^ Nibble3) == LRC)
    {
      return true;
    } 
  }
  return false;
}

LegoPfReceiver LEGOPFRECEIVER;