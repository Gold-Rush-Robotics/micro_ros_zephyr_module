#include <FlexCAN_T4.h>
#include <circular_buffer.h>
#include <imxrt_flexcan.h>
#include <isotp.h>
#include <isotp_server.h>
#include <kinetis_flexcan.h>

#include <FlexCAN_T4.h>

#define LEDPIN 16

FlexCAN_T4FD<CAN3, RX_SIZE_256, TX_SIZE_16> flexCan;

void printInfo(const CANFD_message_t& message, bool received = true, bool result = true)
{
  Serial.println("*******************************");
  Serial.println(received ? "* Frame Received" : "* Frame Send");  
  Serial.println(result ? "* Success" : "* Fail");  
  Serial.println("*");
  Serial.print("* Id: "); Serial.print(message.id, HEX); Serial.println();
  Serial.print("* Data: "); for(int i=0; i<message.len; i++){ Serial.print(message.buf[i], HEX); Serial.print(" "); } Serial.println();
  Serial.println("*");
  Serial.println("*******************************");
}

void frameReceivedHandler(const CANFD_message_t& message)
{
  printInfo(message);
}


int txId = 0x124, rxId = 0x124;

// int txId = 0x321, rxId = 0x123;

bool last_state = true;

void setup(void) {  
  
  Serial.begin(9600);
  
  /************** Config LED **************/
  pinMode(LEDPIN, OUTPUT);    

  delay(500);
  
  /************ Initialize CAN ************/  
  CANFD_timings_t config;
  config.clock = CLK_20MHz;
  config.baudrate = 1000*1000;
  config.baudrateFD = 2000000;
  config.propdelay = 190;
  config.bus_length = 1;
  config.sample = 70;
  flexCan.begin();
  flexCan.setRegions(64);
  flexCan.setBaudRate(config);
  
  // flexCan.setBaudRate(500000);
}

void loop() {  

  static unsigned long timeout = millis();
  
  CANFD_message_t message;
  
  int result;
  static int ctr = 0;
    
  message.id = txId;
  message.len = 8;
  message.buf[0] = 105;
  message.buf[1] = 105;
  message.buf[2] = 105;
  message.buf[3] = 105;
  message.buf[4] = 105;
  message.buf[5] = 105;
  message.buf[6] = 105;
  message.buf[7] = 105;

  if(millis() - timeout >= 500)
  {
    timeout = millis();
      
    result = flexCan.write(message);
    printInfo(message, false, result);
      
    flexCan.mailboxStatus();
    
    /************ Blink LED ************/

    digitalWrite(LEDPIN, last_state);
    last_state= !last_state;

      
  }

  flexCan.events();

  /************ Call event handlers ************/
  if(flexCan.read(message))frameReceivedHandler(message);
}
// bool last_state = true;

// void setup(void){
//   pinMode(31, OUTPUT);    

//   delay(500);

// }
// void loop(){
//   digitalWrite(31, last_state);
//   delay(500);
//   last_state= !last_state;
// }