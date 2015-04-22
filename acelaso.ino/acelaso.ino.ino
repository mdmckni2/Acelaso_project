#include <SI114.h>
#include <Si114_defs.h>
#include <Adafruit_FRAM_I2C.h>
#include <Adafruit_TMP006.h>
#include <Adafruit_Sensor.h>
#include <RFduinoBLE.h>
#include <Wire.h>
#include <pcf8574.h>


int TempAddress = 64; // I2C address of TMP006
int HRAddress = 96;  //I2C Address of SI1146
int FRAMAddress = 80; //I2C Address for FRAM
int interval = 500; //500 ms between advertisement transmission
int ExpanderAddress = B0111000;
int LED1_R = 1;
int LED1_B = 2;
int LED1_G = 4;

int samples = TMP006_CFG_8SAMPLE; // # of samples per reading, can be 1/2/4/8/16

Adafruit_TMP006 tmp006;
Adafruit_FRAM_I2C fram     = Adafruit_FRAM_I2C();
uint16_t          framAddr = 0;

char state = 0;
#define RESET 1
#define POLL_SENSORS 1
#define BUTTON_PRESS 2
#define BLUETOOTH 3
#define GPIO_INT 3


void setup()
{
  override_uart_limit = true;

  //Wire.begin(); // join i2c bus (address optional for master)
  Wire.beginOnPins (6u, 5u);

  Serial.begin(9600);
  //Serial.begin(57600);
  Serial.println("Waiting for connection...");

  //Check to see if temperature sensor is found
//  if (! tmp006.begin()) {
//    Serial.println("No temperature sensor found");
//    while (1);
//  }

  //GPIO Expander Interrupt Pin
  pinMode(2, INPUT); // set pin 2 to input
  RFduino_pinWake(2, HIGH); // configures pin 3 to wake up device on a low signal

  //Setup pin 3 for button press
  pinMode(3, INPUT); // set pin 3 to input
  RFduino_pinWake(3, LOW); // configures pin 3 to wake up device on a low signal

  RFduinoBLE.advertisementData = "BLETest";
  RFduinoBLE.advertisementInterval = interval;
  RFduinoBLE.deviceName = "ACELASO";

  //  //Check to ensure that FRAM device is found
  //  if (fram.begin()) {  // you can stick the new i2c addr in here, e.g. begin(0x51);
  //    Serial.println("Found I2C FRAM");
  //  } else {
  //    Serial.println("No I2C FRAM found ... check your connections\r\n");
  //    while (1);
  //  }
}

// I2C routines to talk to 8574 and 8574A
//void expanderSetInput(int i2caddr, byte dir) {
//  Wire.beginTransmission(i2caddr);
//  Wire.write(dir);  // outputs high for input
//  Wire.endTransmission();
//}
//
//byte expanderRead(int i2caddr) {
//  int _data = -1;
//  Wire.requestFrom(i2caddr, 1);
//  if (Wire.available()) {
//    _data = Wire.read();
//  }
//  return _data;
//}

void expanderWrite(int i2caddr, byte data)
{
  Wire.beginTransmission(i2caddr);
  Wire.write(data);
  Wire.endTransmission();
}

void RFduinoBLE_onConnect()
{
  Serial.println("RFduino BLE connection successful");
  LED_ON();
}

void RFduinoBLE_onDisconnect()
{
  Serial.println("RFduino BLE disconnected");
  LED_OFF();
}

void RFduinoBLE_onAdvertisement(bool start){
  
}

void LED_ON() {
  expanderWrite(ExpanderAddress, 0xFF);
  //  expanderWrite(LED1_R, 2);
  //  expanderWrite(LED1_R, 4);
}

void LED_OFF() {
  expanderWrite(ExpanderAddress, 0x00);
  //  expanderWrite(LED1_R, 0);
  //  expanderWrite(LED1_R, 0);
}

void loop() {
  
  //Detect button Press
  if (RFduino_pinWoke(3)) {
    state = BUTTON_PRESS;
    RFduino_resetPinWake(3); // reset state of pin that caused wakeup (Must do this)
  }

  //This is from GPIO Expander Interrupt
  if (RFduino_pinWoke(2)) {
    state = GPIO_INT;
    RFduino_resetPinWake(2); // reset state of pin that caused wakeup (Must do this)
  }

  //Poll Sensors if time expired
  if (state == POLL_SENSORS) {
    while (RFduinoBLE.radioActive)
      //Heart Rate Monitor Read
      Wire.beginTransmission(HRAddress);  // transmit to SI1146 Heart Rate Monitor device #96 (0x60)

    // Grab temperature measurements and print them.
    float objt = tmp006.readObjTempC();
    Serial.print("Object Temperature: "); Serial.print(objt); Serial.println("*C");
    float diet = tmp006.readDieTempC();
    Serial.print("Die Temperature: "); Serial.print(diet); Serial.println("*C");
    RFduinoBLE.sendFloat(objt);
    //---> Insert into FRAM

    //Galvanic Skin Response Read
    float gsr = analogRead(4);
    Serial.print("Galvanic Skin Response: "); Serial.println(gsr);
    RFduinoBLE.sendFloat(gsr);
    //---> Insert into FRAM

    //Wire.beginTransmission(FRAMAddress); //transmit to FRAM device #160 (0x50)

    //Reset State
    state = RESET;
  }

  else if (state == BUTTON_PRESS) {
    float temp = RFduino_temperature(FAHRENHEIT); // returns temperature in Celsius and stores in float temp
    Serial.print("RFduino Temperature: "); Serial.print(temp); Serial.println("*F");
    LED_ON();
    delay(2000);
    LED_OFF();
    //Reset State
    state = BUTTON_PRESS;
  }

  else if (state == BLUETOOTH) {
    RFduinoBLE.begin();

    //Reset State
    state = RESET;
  }

  //Sleep for 5 minutes or until interrupt
  RFduino_ULPDelay(MINUTES(1));

}
