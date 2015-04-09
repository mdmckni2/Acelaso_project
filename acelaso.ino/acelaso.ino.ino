#include <RFduinoBLE.h>
#include "Adafruit_FRAM_I2C.h"

// send 500 20 byte buffers = 10000 bytes
int packets = 500; 

// flag used to start sending
int flag = false;

// variables used in packet generation
int ch;
int packet;
int start;
int TempAddress = 64; // I2C address of TMP006
int HRAddress = 96;  //I2C Address of SI1146
int FRAMReadAddress=161; //I2C Address for FRAM Read
int FRAMWriteAddress=160;  //I2C Address for FRAM Write
int samples = TMP006_CFG_8SAMPLE; // # of samples per reading, can be 1/2/4/8/16

void setup() {
  Wire.begin(); // join i2c bus (address optional for master)
  Serial.begin(57600);
  Serial.println("Waiting for connection...");
  RFduinoBLE.begin();
}

void RFduinoBLE_onConnect() {
  packet = 0;
  ch = 'A';
  start = 0;
  flag = true;
  Serial.println("Sending");
  // first send is not possible until the iPhone completes service/characteristic discovery
}

void loop() {
  RFduino_ULPDelay( SECONDS(1) );
  
  //Heart Rate Monitor
  Wire.beginTransmission(HRAddress);  // transmit to SI1146 Heart Rate Monitor device #96 (0x60)
  
  Wire.beginTransmission(TempAddress); // transmit to TMP006 Temp Sensor device #64 (0x40)
 
  
  Wire.beginTransmission(FRAMWriteAddress); //transmit to FRAM (Write Address) device #160 (0xa0)
 
                               

  if (flag)
  {
    // generate the next packet
    char buf[20];
    for (int i = 0; i < 20; i++)
    {
      buf[i] = ch;
      ch++;
      if (ch > 'Z')
        ch = 'A';
    }
    
    // send is queued (the ble stack delays send to the start of the next tx window)
    while (! RFduinoBLE.send(buf, 20))
      ;  // all tx buffers in use (can't send - try again later)

    if (! start)
      start = millis();

    packet++;
    if (packet >= packets)
    {
      int end = millis();
      float secs = (end - start) / 1000.0;
      int bps = ((packets * 20) * 8) / secs; 
      Serial.println("Finished");
      Serial.println(start);
      Serial.println(end);
      Serial.println(secs);
      Serial.println(bps / 1000.0);
      flag = false;
    }
  }
}
