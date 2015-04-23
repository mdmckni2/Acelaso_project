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
int samples = TMP006_CFG_8SAMPLE; // # of samples per reading, can be 1/2/4/8/16
int lastButtonState = LOW;
long button_holdtime = 2000;
int ledflash = 5;
int BLE_State = 0;

Adafruit_TMP006 tmp006;
Adafruit_FRAM_I2C fram     = Adafruit_FRAM_I2C();
uint16_t framAddr = 0;

char state = 0;
#define RESET 0
#define POLL_SENSORS 1
#define BUTTON_PRESS 2
#define BLUETOOTH 3

#define ON 1
#define OFF 0

//------------------------------------------------------------------
const int SAMPLES_TO_AVERAGE = 5; // samples for smoothing 1 to 10 seem useful 5 is default
// increase for smoother waveform (with less resolution - slower!)
int binOut; // 1 or 0 depending on state of heartbeat
int BPM;
unsigned long red;        // read value from visible red LED
unsigned long IR1;        // read value from infrared LED1
unsigned long IR2;       // read value from infrared LED2
unsigned long IR_total;     // IR LED reads added together

const int portForSI114 = 1;        // change to the JeeNode port number used
PortI2C myBus (portForSI114);
PulsePlug pulse (myBus);
//------------------------------------------------------------------


void setup()
{  
  override_uart_limit = true;

  //Wire.begin(); // join i2c bus (address optional for master)
  Wire.beginOnPins (6u, 5u); //SCL,SDA

  Serial.begin(9600);
  //Serial.begin(57600);
  Serial.println("Waiting for connection...");

  //Check to see if temperature sensor is found
  //  if (! tmp006.begin()) {
  //    Serial.println("No temperature sensor found");
  //    while (1);
  //  }

  //Setup pin 3 for button press
  pinMode(3, INPUT); // set pin 3 to input
  RFduino_pinWake(3, LOW); // configures pin 3 to wake up device on a low signal

  RFduinoBLE.advertisementData = "BLETest";
  RFduinoBLE.advertisementInterval = interval;
  RFduinoBLE.deviceName = "ACELASO";
  delay(2000);

  //Check to ensure that FRAM device is found
  //  if (fram.begin()) {  // you can stick the new i2c addr in here, e.g. begin(0x51);
  //    Serial.println("Found I2C FRAM");
  //  } else {
  //    Serial.println("No I2C FRAM found ... check your connections\r\n");
  //    while (1);
  //  }

  //Init Heart Rate Sensor
  //initPulseSensor();
  
    LED_OFF();
}

void expanderWrite(int i2caddr, byte data)
{
  Wire.beginTransmission(i2caddr);
  Wire.write(data);
  Wire.endTransmission();
}

void RFduinoBLE_onConnect() {
  Serial.println("RFduino BLE connection successful");
  BLUE_LED_ON();
}

void RFduinoBLE_onDisconnect() {
  Serial.println("RFduino BLE disconnected");
  RED_LED_ON();
  //LED_OFF();

}

void RFduinoBLE_onAdvertisement(bool start) {
  if (start) {
    Serial.println("RFduino BLE advertising");
    GREEN_LED_ON();
  }
  else {
   LED_OFF(); 
  }
}

void RFduinoBLE_onReceive(char *data, int len) {
  uint8_t myByte = data[0]; // store first char in array to myByte
  Serial.print("RFduino BLE Received: ");
  Serial.println(myByte); // print myByte via serial
}

void RFduinoBLE_onRSSI(int rssi) {
  Serial.print("RFduino BLE RSSI: ");
  Serial.println(rssi); // print rssi value via serial
}

void LED_OFF() {
  expanderWrite(ExpanderAddress, 0xFF);
}

void LED_ON() {
  expanderWrite(ExpanderAddress, 0x00);
}

void RED_LED_ON() {
  expanderWrite(ExpanderAddress, 0xFE);
}

void BLUE_LED_ON() {
  expanderWrite(ExpanderAddress, 0xFD);
}

void GREEN_LED_ON() {
  expanderWrite(ExpanderAddress, 0xFB);
}

void LED_FLASH() {      //Flash Green LED for a few seconds
  int i;
  for (i = 0; i < ledflash; i++) {
    expanderWrite(ExpanderAddress, 0xFB);
    delay(400);
    expanderWrite(ExpanderAddress, 0xFF);
    delay(400);
  }
}

//--------------------------------------------------------------------------------------------
// New Code Section for Heart Rate

float smooth(float data, float filterVal, float smoothedVal) {

  if (filterVal > 1) {     // check to make sure param's are within range
    filterVal = .99;
  }
  else if (filterVal <= 0.0) {
    filterVal = 0.01;
  }

  smoothedVal = (data * (1.0 - filterVal)) + (smoothedVal  *  filterVal);
  return smoothedVal;
}

void readPulseSensor() {

  static int foundNewFinger, red_signalSize, red_smoothValley;
  static long red_valley, red_Peak, red_smoothRedPeak, red_smoothRedValley,
         red_HFoutput, red_smoothPeak; // for PSO2 calc
  static  int IR_valley = 0, IR_peak = 0, IR_smoothPeak, IR_smoothValley, binOut, lastBinOut, BPM;
  static unsigned long lastTotal, lastMillis, IRtotal, valleyTime = millis(), lastValleyTime = millis(), peakTime = millis(), lastPeakTime = millis(), lastBeat, beat;
  static float IR_baseline, red_baseline, IR_HFoutput, IR_HFoutput2, shiftedOutput, LFoutput, hysterisis;

  unsigned long total = 0, start;
  int i = 0;
  int IR_signalSize;
  red = 0;
  IR1 = 0;
  IR2 = 0;
  total = 0;
  start = millis();

  while (i < SAMPLES_TO_AVERAGE) {
    pulse.fetchLedData();
    red += pulse.ps1;
    IR1 += pulse.ps2;
    IR2 += pulse.ps3;
    i++;
  }

  red = red / i;  // get averages
  IR1 = IR1 / i;
  IR2 = IR2 / i;
  total =  IR1 + IR2 + red;  // red excluded
  IRtotal = IR1 + IR2;

  // except this one for Processing heartbeat monitor
  // comment out all the bottom print lines

  if (lastTotal < 20000L && total > 20000L) foundNewFinger = 1;  // found new finger!

  lastTotal = total;

  // if found a new finger prime filters first 20 times through the loop
  if (++foundNewFinger > 25) foundNewFinger = 25;   // prevent rollover

  if ( foundNewFinger < 20) {
    IR_baseline = total - 200;   // take a guess at the baseline to prime smooth filter
    Serial.println("found new finger");
  }

  else if (total > 20000L) {   // main running function

    // baseline is the moving average of the signal - the middle of the waveform
    // the idea here is to keep track of a high frequency signal, HFoutput and a
    // low frequency signal, LFoutput
    // The LF signal is shifted downward slightly downward (heartbeats are negative peaks)
    // The high freq signal has some hysterisis added.
    // When the HF signal crosses the shifted LF signal (on a downward slope),
    // we have found a heartbeat.
    IR_baseline = smooth(IRtotal, 0.99, IR_baseline);   //
    IR_HFoutput = smooth((IRtotal - IR_baseline), 0.2, IR_HFoutput);    // recycling output - filter to slow down response

    red_baseline = smooth(red, 0.99, red_baseline);
    red_HFoutput = smooth((red - red_HFoutput), 0.2, red_HFoutput);

    // beat detection is performed only on the IR channel so
    // fewer red variables are needed

    IR_HFoutput2 = IR_HFoutput + hysterisis;
    LFoutput = smooth((IRtotal - IR_baseline), 0.95, LFoutput);
    // heartbeat signal is inverted - we are looking for negative peaks
    shiftedOutput = LFoutput - (IR_signalSize * .05);

    if (IR_HFoutput  > IR_peak) IR_peak = IR_HFoutput;
    if (red_HFoutput  > red_Peak) red_Peak = red_HFoutput;

    // default reset - only if reset fails to occur for 1800 ms
    if (millis() - lastPeakTime > 1800) { // reset peak detector slower than lowest human HB
      IR_smoothPeak =  smooth((float)IR_peak, 0.6, (float)IR_smoothPeak);  // smooth peaks
      IR_peak = 0;

      red_smoothPeak =  smooth((float)red_Peak, 0.6, (float)red_smoothPeak);  // smooth peaks
      red_Peak = 0;

      lastPeakTime = millis();
    }

    if (IR_HFoutput  < IR_valley)   IR_valley = IR_HFoutput;
    if (red_HFoutput  < red_valley)   red_valley = red_HFoutput;

    /*      if (IR_valley < -1500){
              IR_valley = -1500;  // ditto above
              Serial.println("-1500");
          }
          if (red_valley < -1500) red_valley = -1500;  // ditto above  */



    if (millis() - lastValleyTime > 1800) { // insure reset slower than lowest human HB
      IR_smoothValley =  smooth((float)IR_valley, 0.6, (float)IR_smoothValley);  // smooth valleys
      IR_valley = 0;
      lastValleyTime = millis();
    }

    //     IR_signalSize = IR_smoothPeak - IR_smoothValley;  // this the size of the smoothed HF heartbeat signal
    hysterisis = constrain((IR_signalSize / 15), 35, 120) ;  // you might want to divide by smaller number
    // if you start getting "double bumps"

    // Serial.print(" T  ");
    // Serial.print(IR_signalSize);

    if  (IR_HFoutput2 < shiftedOutput) {
      // found a beat - pulses are valleys
      lastBinOut = binOut;
      binOut = 1;
      //   Serial.println("\t1");
      hysterisis = -hysterisis;
      IR_smoothValley =  smooth((float)IR_valley, 0.99, (float)IR_smoothValley);  // smooth valleys
      IR_signalSize = IR_smoothPeak - IR_smoothValley;
      IR_valley = 0x7FFF;

      red_smoothValley =  smooth((float)red_valley, 0.99, (float)red_smoothValley);  // smooth valleys
      red_signalSize = red_smoothPeak - red_smoothValley;
      red_valley = 0x7FFF;

      lastValleyTime = millis();

    }
    else {
      Serial.println("\t0");
      lastBinOut = binOut;
      binOut = 0;
      IR_smoothPeak =  smooth((float)IR_peak, 0.99, (float)IR_smoothPeak);  // smooth peaks
      IR_peak = 0;

      red_smoothPeak =  smooth((float)red_Peak, 0.99, (float)red_smoothPeak);  // smooth peaks
      red_Peak = 0;
      lastPeakTime = millis();
    }

    if (lastBinOut == 1 && binOut == 0) {
      Serial.println(binOut);
    }

    if (lastBinOut == 0 && binOut == 1) {
      lastBeat = beat;
      beat = millis();
      BPM = 60000 / (beat - lastBeat);
      Serial.print(binOut);
      Serial.print("\t BPM ");
      Serial.print(BPM);
      Serial.print("\t IR ");
      Serial.print(IR_signalSize);
      Serial.print("\t PSO2 ");
      Serial.println(((float)red_baseline / (float)(IR_baseline / 2)), 3);
    }
  }
}

void initPulseSensor() {

  pulse.setReg(PulsePlug::HW_KEY, 0x17);
  // pulse.setReg(pulse::COMMAND, pulse::RESET_Cmd);

  Serial.print("PART: ");
  Serial.print(pulse.getReg(PulsePlug::PART_ID));
  Serial.print(" REV: ");
  Serial.print(pulse.getReg(PulsePlug::REV_ID));
  Serial.print(" SEQ: ");
  Serial.println(pulse.getReg(PulsePlug::SEQ_ID));

  pulse.setReg(PulsePlug::INT_CFG, 0x03);       // turn on interrupts
  pulse.setReg(PulsePlug::IRQ_ENABLE, 0x10);    // turn on interrupt on PS3
  pulse.setReg(PulsePlug::IRQ_MODE2, 0x01);     // interrupt on ps3 measurement
  pulse.setReg(PulsePlug::MEAS_RATE, 0x84);     // see datasheet
  pulse.setReg(PulsePlug::ALS_RATE, 0x08);      // see datasheet
  pulse.setReg(PulsePlug::PS_RATE, 0x08);       // see datasheet
  pulse.setReg(PulsePlug::PS_LED21, 0x66 );      // LED current for LEDs 1 (red) & 2 (IR1)
  pulse.setReg(PulsePlug::PS_LED3, 0x06);        // LED current for LED 3 (IR2)

  Serial.print( "PS_LED21 = ");
  Serial.println(pulse.getReg(PulsePlug::PS_LED21), BIN);
  Serial.print("CHLIST = ");
  Serial.println(pulse.readParam(0x01), BIN);

  pulse.writeParam(PulsePlug::PARAM_CH_LIST, 0x77);         // all measurements on

  // increasing PARAM_PS_ADC_GAIN will increase the LED on time and ADC window
  // you will see increase in brightness of visible LED's, ADC output, & noise
  // datasheet warns not to go beyond 4 because chip or LEDs may be damaged
  pulse.writeParam(PulsePlug::PARAM_PS_ADC_GAIN, 0x00);

  pulse.writeParam(PulsePlug::PARAM_PSLED12_SELECT, 0x21);  // select LEDs on for readings see datasheet
  pulse.writeParam(PulsePlug::PARAM_PSLED3_SELECT, 0x04);   //  3 only
  pulse.writeParam(PulsePlug::PARAM_PS1_ADCMUX, 0x03);      // PS1 photodiode select
  pulse.writeParam(PulsePlug::PARAM_PS2_ADCMUX, 0x03);      // PS2 photodiode select
  pulse.writeParam(PulsePlug::PARAM_PS3_ADCMUX, 0x03);      // PS3 photodiode select

  pulse.writeParam(PulsePlug::PARAM_PS_ADC_COUNTER, B01110000);    // B01110000 is default
  pulse.setReg(PulsePlug::COMMAND, PulsePlug::PSALS_AUTO_Cmd);     // starts an autonomous read loop
  Serial.println(pulse.getReg(PulsePlug::CHIP_STAT), HEX);
  Serial.print("end init");
}

//--------------------------------------------------------------------------------------------
//int SI1146_Test(int i2caddr) {
//  Wire.beginTransmission(i2caddr);
//  char id = Wire.read(SI114_REG_PART_ID);
//  Wire.endTransmission();
//  if (id == 0x46)
//    return 1; // look for SI1145
//  //DEVICE FOUND Reset SI1146 Registers
//  return 0;
//}

//--------------------------------------------------------------------------------------------

void loop() {
  Serial.println("Top of Main");

  //Detect button Press
  if (RFduino_pinWoke(3)) {
    RFduino_resetPinWake(3); // reset state of pin that caused wakeup (Must do this)
    Serial.println("Button Press!");
    state = BLUETOOTH;
    delay(2500);
  }

  //Poll Sensors if time expired
  if (state == POLL_SENSORS) {

    //Do nothing if BLE is transmitting
    while (RFduinoBLE.radioActive)
      ;

    //Heart Rate Monitor Read
    Wire.beginTransmission(HRAddress);  // transmit to SI1146 Heart Rate Monitor device #96 (0x60)
    //readPulseSensor();
    //---> Insert into FRAM


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

    //Reset State
    state = POLL_SENSORS;
  }

  else if (state == BUTTON_PRESS) {
    //float temp = RFduino_temperature(FAHRENHEIT); // returns temperature in Celsius and stores in float temp
    //Serial.print("RFduino Temperature: "); Serial.print(temp); Serial.println("*F");

    RED_LED_ON();
    delay(1000);
    LED_OFF();
    delay(1000);

    Serial.println("Heart Rate Read Attempt");
    readPulseSensor();

    //    SI1146_Test(HRAddress);

    RED_LED_ON();
    delay(1000);
    LED_OFF();
    delay(1000);

    state = RESET;
  }

  else if (state == BLUETOOTH) {
    if (BLE_State == OFF) {
      Serial.println("Bluetooth Begin");
      RFduinoBLE.begin();
      delay(2000);
      Serial.println("Bluetooth On");
      BLE_State = ON;
    }

    else if (BLE_State == ON) {
      RFduinoBLE.end();
      Serial.println("Bluetooth ending");
      BLE_State = OFF;
      LED_OFF();
    }

    state = RESET;
  }

  Serial.println("Bottom of Main");
  //Sleep for 5 minutes or until interrupt
  RFduino_ULPDelay(MINUTES(1));

}


