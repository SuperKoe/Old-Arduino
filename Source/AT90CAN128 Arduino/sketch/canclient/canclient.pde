#include <can_lib.h>
#define MY_ID_TAG 81              //- 0x80 hexa
//#define CAN_BAUDRATE   CAN_AUTOBAUD
//#define CAN_BAUDRATE   250
st_cmd_t sensor_message;
unsigned char sensor_buffer[8];


void setup()   
{  
  Serial.begin(9600);
  CAN.set_baudrate(CAN_AUTOBAUD);
  CAN.init(0);  
  
  // --- Init Reply data
  sensor_message.pt_data = &sensor_buffer[0];
}

void loop()                     
{
  sensor_buffer[0] = 'H';
  sensor_buffer[1] = 'e';
  sensor_buffer[2] = 'l';
  sensor_buffer[3] = 'l';
  sensor_buffer[4] = 'o';
  sensor_buffer[5] = '2';
		
  // --- Auto-reply Command
  sensor_message.ctrl.ide = 0;            //- CAN 2.0A
  sensor_message.dlc = 8;                 //- Message with 8-byte data
  sensor_message.id.std = MY_ID_TAG;
  sensor_message.cmd = CMD_REPLY_MASKED;

  // --- Enable reply
  while(CAN.cmd(&sensor_message) != CAN_CMD_ACCEPTED);
  // --- Wait for Reply completed
  while(CAN.get_status(&sensor_message) == CAN_STATUS_NOT_COMPLETED);
  
  Serial.println("succses");
}
