
#include <SI114.h>
#include <Si114_defs.h>
#include <Adafruit_FRAM_I2C.h>
#include <Adafruit_TMP006.h>
#include <Adafruit_Sensor.h>
#include <RFduinoBLE.h>
#include <Wire.h>


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
int FRAMAddress=80; //I2C Address for FRAM 
int ExpAddress=32;

int samples = TMP006_CFG_8SAMPLE; // # of samples per reading, can be 1/2/4/8/16

Adafruit_TMP006 tmp006;
Adafruit_FRAM_I2C fram     = Adafruit_FRAM_I2C();
uint16_t          framAddr = 0;

void setup() 
{
  Wire.begin(); // join i2c bus (address optional for master)
  expanderSetInput(ExpAddress, 0xFF);
  Serial.begin(57600);
  Serial.println("Waiting for connection...");
  RFduinoBLE.begin();

  //Check to see if temperature sensor is found
  if (! tmp006.begin()) {
    Serial.println("No temperature sensor found");
    while (1);
  }
  
  //Check to ensure that FRAM device is found
  if (fram.begin()) {  // you can stick the new i2c addr in here, e.g. begin(0x51);
    Serial.println("Found I2C FRAM");
  } else {
    Serial.println("No I2C FRAM found ... check your connections\r\n");
    while (1);
  }
  
}
  
  // I2C routines to talk to 8574 and 8574A
  void expanderSetInput(int i2caddr, byte dir) {
  Wire.beginTransmission(ExpAddress);
  Wire.write(dir);  // outputs high for input
  Wire.endTransmission();    
}

byte expanderRead(int i2caddr) {
  int _data = -1;
  Wire.requestFrom(i2caddr, 1);
  if(Wire.available()) {
    _data = Wire.read();
  }
  return _data;
}

void expanderWrite(int i2caddr, byte data)
{
  Wire.beginTransmission(i2caddr);
  Wire.write(data);
  Wire.endTransmission();   
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
  
  //Heart Rate Monitor Read 
  Wire.beginTransmission(HRAddress);  // transmit to SI1146 Heart Rate Monitor device #96 (0x60)

 // Grab temperature measurements and print them.
  float objt = tmp006.readObjTempC();
  Serial.print("Object Temperature: "); Serial.print(objt); Serial.println("*C");
  float diet = tmp006.readDieTempC();
  Serial.print("Die Temperature: "); Serial.print(diet); Serial.println("*C");
  
  //Galvanic Skin Response Read
  float gsr = analogRead(4);
  Serial.print("Galvanic Skin Response: "; Serial.print(gsr); 
 
  
  Wire.beginTransmission(ExpAddress);// transmit to GPIIO device #32 (0x20)

  Wire.beginTransmission(FRAMAddress); //transmit to FRAM device #160 (0x50)
  
  
 
                               

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
