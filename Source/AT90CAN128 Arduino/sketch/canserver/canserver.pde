#define ID_TAG_SLAVE 80
#define NB_TARGET   2

unsigned char tx_remote_buffer[8];
st_cmd_t tx_remote_msg;

unsigned char response_buffer[8];
st_cmd_t response_msg;

void setup()   
{    
  Serial.begin(9600);  
  CAN.set_baudrate(250);
  CAN.init(0);
  
  // --- Init variables
  tx_remote_msg.pt_data = &tx_remote_buffer[0];
  tx_remote_msg.status = 0;

  response_msg.pt_data = &response_buffer[0];
  response_msg.status = 0;
}

void loop()                     
{ 
  int i,j;
  for(j = 0; j < NB_TARGET; j++)
  {
    delay(2000);
  
    // --- Init Rx Commands
    for(i = 0; i < 8; i++) 
      response_buffer[i]=0;
    response_msg.id.std = ID_TAG_SLAVE +j; // id
    response_msg.ctrl.ide = 0;
    response_msg.ctrl.rtr = 0;
    //response_msg.dlc = 8;
    response_msg.cmd = CMD_RX_DATA_MASKED;
  
    // --- Rx Command
    while(CAN.cmd(&response_msg) != CAN_CMD_ACCEPTED);

    // --- Init Tx Commands
    for(i = 0; i < 8; i++) 
      tx_remote_buffer[i]=0;
    tx_remote_msg.id.std = ID_TAG_SLAVE+j;
    tx_remote_msg.ctrl.ide = 0;
    tx_remote_msg.ctrl.rtr = 1;
    tx_remote_msg.dlc = 8;
    tx_remote_msg.cmd = CMD_TX_REMOTE;
    // --- Tx Command
    while(CAN.cmd(&tx_remote_msg) != CAN_CMD_ACCEPTED);
  
    // --- Wait for Tx remote completed
    while(CAN.get_status(&tx_remote_msg) == CAN_STATUS_NOT_COMPLETED)
      delay(50);
  
    if (CAN.get_status(&response_msg) == CAN_STATUS_COMPLETED)
    {
      Serial.print("Target_ID = ");
      Serial.println(response_msg.id.std); 
      for(i = 0; i < 8; i++) 
      {
        Serial.print(response_buffer[i]);    
      }
      Serial.println();
    }
    else
    {
      response_msg.cmd = CMD_ABORT;
      while (CAN.cmd(&response_msg) != CAN_CMD_ACCEPTED);
    }
  }
}
