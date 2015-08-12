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
#define interval 500 //500 ms between advertisement transmission
#define ExpanderAddress B0111000
// int samples = TMP006_CFG_8SAMPLE; // # of samples per reading, can be 1 / 2 / 4 / 8 / 16
#define ledflash 5

// STATES FOR FSM
#define RESET 0
#define POLL_SENSORS 1
#define BUTTON_PRESS 2
#define BLUETOOTH_ENABLE 3
#define TRANSMIT_DATA 4
#define CONNECTED 5
#define WAIT_TO_CONNECT 6
#define FRAM_TESTING 9

#define FRAM_SIZE (32*1024) 

#define ON 1
#define OFF 0

Adafruit_TMP006 tmp006(TempAddress);
Adafruit_FRAM_I2C fram = Adafruit_FRAM_I2C();
uint16_t framAddr = 0;

char state = POLL_SENSORS;
char prev_state = RESET;
bool connectedBLE = false;
int BLE_State = 0;

#define SAMPLES_TO_AVERAGE 5 // samples for smoothing 1 to 10 seem useful 5 is default
// increase for smoother waveform (with less resolution - slower!)
int binOut; // 1 or 0 depending on state of heartbeat
int BPM;
int IR_signalSize;  //THIS COULD BREAK THINGS
float pso2;
unsigned long red;        // read value from visible red LED
unsigned long IR1;        // read value from infrared LED1
unsigned long IR2;       // read value from infrared LED2
unsigned long IR_total;     // IR LED reads added together
unsigned int resp, als_vis, als_ir, ps1, ps2, ps3;

float temp_data_float[1] = {0};

uint16_t fram_current_address = 0;
uint16_t fram_bluetooth_address = 0;

//--------------------------------------------------------------------------------------------
// program setup

void setup()
{
  override_uart_limit = true;

  //Wire.begin(); // join i2c bus (address optional for master)
  Wire.beginOnPins (6u, 5u); //SCL,SDA

  Serial.begin(9600);
  //Serial.begin(57600);
  Serial.println("Waiting for connection...");

  //Check to see if temperature sensor is found
  if (! tmp006.begin()) {
    Serial.println("No temperature sensor found");
//    while (1);
  }
  else{
        Serial.println("Found temperature sensor");
  }

  //Setup pin 3 for button press
  pinMode(3, INPUT_PULLUP); // set pin 3 to input
  RFduino_pinWake(3, LOW); // configures pin 3 to wake up device on a low signal

  RFduinoBLE.advertisementData = "Data";
  RFduinoBLE.advertisementInterval = interval;
  RFduinoBLE.deviceName = "ACELASO";
//  RFduinoBLE.txPowerLevel = 0;
  delay(2000);

  //Check to ensure that FRAM device is found
  if (fram.begin(FRAMAddress)) {  // you can stick the new i2c addr in here, e.g. begin(0x50);
    Serial.println("Found I2C FRAM");
  } else {
    Serial.println("No I2C FRAM found ... check your connections\r\n");
//    while (1);
  }

  //Init Heart Rate Sensor
  initPulseSensor();
  
  LED_OFF();  
}

//--------------------------------------------------------------------------------------------
// RFduino Bluetooth Setup

void RFduinoBLE_onConnect() {
  Serial.println("RFduino BLE connection successful");
  BLUE_LED_ON();
  connectedBLE = true;
  prev_state = CONNECTED;
  state = TRANSMIT_DATA;
}

void RFduinoBLE_onDisconnect() {
  Serial.println("RFduino BLE disconnected");
//  RED_LED_ON();
  connectedBLE = false;
  LED_OFF();
  state = POLL_SENSORS;
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
  //  Serial.print("RFduino BLE RSSI: ");
  //  Serial.println(rssi); // print rssi value via serial
}

//--------------------------------------------------------------------------------------------
// Expander and LEDs

void expanderWrite(int i2caddr, byte data)
{
  Wire.beginTransmission(i2caddr);
  Wire.write(data);
  Wire.endTransmission();
}

void LED_OFF() {
  expanderWrite(ExpanderAddress, 0xFF);
}

void LED_ON() {
  expanderWrite(ExpanderAddress, 0x00);
}

void GREEN_LED_ON() {
  expanderWrite(ExpanderAddress, 0xF7);
}

void RED_LED_ON() {
  expanderWrite(ExpanderAddress, 0xEF);
}

void BLUE_LED_ON() {
  expanderWrite(ExpanderAddress, 0xDF);
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
// Heart Rate

int SI1146_Test(int i2caddr) {
  int id;
  id = HR_read_reg(SI114_REG_PART_ID, 1);
  Serial.print("Part id: ");  Serial.println(id);
  if (id == 0x46)
    return 1; // look for SI1145
  //DEVICE FOUND Reset SI1146 Registers
  return 0;
}

//This was get reg, num_data is number of bytes to return
char HR_read_reg(unsigned char address, int num_data) // Read a Register
{
  Wire.beginTransmission(HRAddress);
  Wire.write(address);
  Wire.endTransmission();
  Wire.requestFrom(HRAddress, num_data);
  while (Wire.available() < num_data);
  return Wire.read();
}

//This was set reg
void HR_write_reg(byte address, byte val) {  // Write a resigter
  Wire.beginTransmission(HRAddress);
  Wire.write(address);
  Wire.write(val);
  Wire.endTransmission();
}

byte HR_read_param(byte addr) {
  // read from parameter ram
  Wire.beginTransmission(HRAddress);
  Wire.write(SI114_REG_COMMAND);
  Wire.write(0x80 | addr);
  Wire.endTransmission();
  return HR_read_reg(SI114_REG_PARAM_RD, 1);
}

void HR_write_param(byte addr, byte val) {
  // write to parameter ram
  Wire.beginTransmission(HRAddress);
  Wire.write(SI114_REG_PARAM_WR);
  Wire.write(val);
  // auto-increments into SI114_REG_COMMAND
  Wire.write(0xA0 | addr); // PARAM_SET
  Wire.endTransmission();
}

void fetchLedData() {
  //  // read only the LED registers as lsb-msb pairs of bytes
  //  Wire.beginTransmission(HRAddress);
  //  Wire.write(SI114_REG_PS1_DATA0);
  //  Wire.endTransmission();
  //  byte* p = (byte*) &ps1;
  //  Wire.requestFrom(HRAddress, 6);
  //  while (Wire.available() < 6);
  //  p = (byte*) Wire.read();
  //  Wire.endTransmission();

  // read only the LED registers as lsb-msb pairs of bytes
  char PS1_0, PS1_1, PS2_0, PS2_1, PS3_0, PS3_1;
  PS1_0 = HR_read_reg(SI114_REG_PS1_DATA0, 1);
  PS1_1 = HR_read_reg(SI114_REG_PS1_DATA1, 1);
  PS2_0 = HR_read_reg(SI114_REG_PS2_DATA0, 1);
  PS2_1 = HR_read_reg(SI114_REG_PS2_DATA1, 1);
  PS3_0 = HR_read_reg(SI114_REG_PS3_DATA0, 1);
  PS3_1 = HR_read_reg(SI114_REG_PS3_DATA1, 1);

  ps1 = (int) ((PS1_1 << 8) | PS1_0);
  ps2 = (int) ((PS2_1 << 8) | PS2_0);
  ps3 = (int) ((PS3_1 << 8) | PS3_0);
}

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
  static  int IR_valley = 0, IR_peak = 0, IR_smoothPeak, IR_smoothValley,
              binOut, lastBinOut;
  static unsigned long lastTotal, lastMillis, IRtotal, valleyTime =
    millis(), lastValleyTime = millis(), peakTime = millis(), lastPeakTime =
                                 millis(), lastBeat, beat;
  static float IR_baseline, red_baseline, IR_HFoutput, IR_HFoutput2,
         shiftedOutput, LFoutput, hysterisis;

  unsigned long total = 0, start;
  int i = 0;
  red = 0;
  IR1 = 0;
  IR2 = 0;
  total = 0;
  start = millis();

  while (i < SAMPLES_TO_AVERAGE) {
    fetchLedData();
    red += ps1;
    IR1 += ps2;
    IR2 += ps3;
    i++;
    /*
    Serial.print("i: "); Serial.print(i);
    Serial.print(" ps1: "); Serial.print(ps1);
    Serial.print(" ps2: "); Serial.print(ps2);
    Serial.print(" ps3: "); Serial.println(ps3);
    */
  }

  red = red / i;  // get averages
  IR1 = IR1 / i;
  IR2 = IR2 / i;
  total =  IR1 + IR2 + red;  // red excluded
  IRtotal = IR1 + IR2;

  /*
  Serial.print("red ");
  Serial.print(red);
  Serial.print("\t");
  Serial.print("IR1 ");
  Serial.print(IR1);
  Serial.print("\t");
  Serial.print("IR2 ");
  Serial.print(IR2);
  Serial.print("\t");
  Serial.print("total ");
  Serial.println((long)total);
  Serial.print("\t");
  */


  // except this one for Processing heartbeat monitor
  // comment out all the bottom print lines

  if (lastTotal < 20000L && total > 20000L) foundNewFinger = 1;  // found new finger!

  lastTotal = total;

  // if found a new finger prime filters first 20 times through the loop
  if (++foundNewFinger > 25) foundNewFinger = 25;   // prevent rollover

  if ( foundNewFinger < 20) {
    IR_baseline = total - 200;   // take a guess at the baseline to prime smooth filter
//    Serial.println("found new finger");
  }

  else if (total > 20000L) {   // main running function

    //Serial.println("HR Main Running Function");

    // baseline is the moving average of the signal - the middle of the waveform
    // the idea here is to keep track of a high frequency signal, HF outputand a
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
      //Serial.println("Problem Here! ");
      IR_smoothPeak =  smooth((float)IR_peak, 0.6, (float)IR_smoothPeak);
      // smooth peaks
      IR_peak = 0;

      red_smoothPeak =  smooth((float)red_Peak, 0.6,
                               (float)red_smoothPeak);  // smooth peaks
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
      IR_smoothValley =  smooth((float)IR_valley, 0.6,
                                (float)IR_smoothValley);  // smooth valleys
      IR_valley = 0;
      lastValleyTime = millis();
    }

    //     IR_signalSize = IR_smoothPeak - IR_smoothValley;  // this the size of the smoothed HF heartbeat signal
    hysterisis = constrain((IR_signalSize / 15), 35, 120) ;  // you might want to divide by smaller number
    // if you start getting "double bumps"

    //Serial.print(" T  ");
    //Serial.print(IR_signalSize);

    if  (IR_HFoutput2 < shiftedOutput) {
      //Serial.println("Found a Beat");
      // found a beat - pulses are valleys
      lastBinOut = binOut;
      binOut = 1;
      //   Serial.println("\t1");
      hysterisis = -hysterisis;
      IR_smoothValley =  smooth((float)IR_valley, 0.99,
                                (float)IR_smoothValley);  // smooth valleys
      IR_signalSize = IR_smoothPeak - IR_smoothValley;
      IR_valley = 0x7FFF;

      red_smoothValley =  smooth((float)red_valley, 0.99,
                                 (float)red_smoothValley);  // smooth valleys
      red_signalSize = red_smoothPeak - red_smoothValley;
      red_valley = 0x7FFF;

      lastValleyTime = millis();

    }
    else {
      // Serial.println("\t0");
      lastBinOut = binOut;
      binOut = 0;
      IR_smoothPeak =  smooth((float)IR_peak, 0.99, (float)IR_smoothPeak);
      // smooth peaks
      IR_peak = 0;

      red_smoothPeak =  smooth((float)red_Peak, 0.99,
                               (float)red_smoothPeak);  // smooth peaks
      red_Peak = 0;
      lastPeakTime = millis();
    }

    if (lastBinOut == 1 && binOut == 0) {
      //Serial.println(binOut);
    }

    if (lastBinOut == 0 && binOut == 1) {
      lastBeat = beat;
      beat = millis();
      BPM = 60000 / (beat - lastBeat);
      pso2 = ((float)red_baseline / (float)(IR_baseline / 2));
//      Serial.print(binOut);
//      Serial.print("\t BPM ");
//      Serial.print(BPM);
//      Serial.print("\t IR ");
//      Serial.print(IR_signalSize);
//      Serial.print("\t PSO2 ");
//      Serial.println(pso2, 3);
    }
  }
}

void HR_EnableLEDs() {
  HR_write_reg(SI114_REG_PS_LED21, 0x66 );      // LED current for LEDs 1 (red) & 2 (IR1)
  HR_write_reg(SI114_REG_PS_LED3, 0x06);        // LED current for LED 3 (IR2)
}

void HR_DisableLEDs() {
  HR_write_reg(SI114_REG_PS_LED21, 0x00 );      // LED current for LEDs 1 (red) & 2 (IR1)
  HR_write_reg(SI114_REG_PS_LED3, 0x00);        // LED current for LED 3 (IR2)
}

void initPulseSensor() {

  HR_write_reg(SI114_REG_HW_KEY, 0x17);
//  HR_write_reg(SI114_REG_COMMAND, B00000001);

//  Serial.print("PART: ");
//  Serial.print(HR_read_reg(SI114_REG_PART_ID, 1));
//  Serial.print(" REV: ");
//  Serial.print(HR_read_reg(SI114_REG_REV_ID, 1));
//  Serial.print(" SEQ: ");
//  Serial.println(HR_read_reg(SI114_REG_SEQ_ID, 1));

  HR_write_reg(SI114_REG_IRQ_CFG, 0x03);       // turn on interrupts
  HR_write_reg(SI114_REG_IRQ_ENABLE, 0x10);    // turn on interrupt on PS3
  HR_write_reg(SI114_REG_IRQ_MODE2, 0x01);     // interrupt on ps3 measurement
  HR_write_reg(SI114_REG_MEAS_RATE, 0x84);     // see datasheet
  HR_write_reg(SI114_REG_ALS_RATE, 0x08);      // see datasheet
  HR_write_reg(SI114_REG_PS_RATE, 0x08);       // see datasheet
  HR_write_reg(SI114_REG_PS_LED21, 0x66 );      // LED current for LEDs 1 (red) & 2 (IR1)
  HR_write_reg(SI114_REG_PS_LED3, 0x06);        // LED current for LED 3 (IR2)

//  Serial.print( "PS_LED21 = ");
//  Serial.println(HR_read_reg(SI114_REG_PS_LED21, 1), BIN);
//  Serial.print("CHLIST = ");
//  Serial.println(HR_read_param(0x01), BIN);

  HR_write_param(SI114_PARAM_CH_LIST, 0x77);         // all measurements on

  // increasing PARAM_PS_ADC_GAIN will increase the LED on time and ADC window
  // you will see increase in brightness of visible LED's, ADC output, & noise
  // datasheet warns not to go beyond 4 because chip or LEDs may be damaged
  HR_write_param(SI114_PARAM_PS_ADC_GAIN, 0x04); // originally set at 1

  HR_write_param(SI114_PARAM_PSLED12_SELECT, 0x21);  // select LEDs on for readings see datasheet
  HR_write_param(SI114_PARAM_PSLED3_SELECT, 0x04);   //  3 only
  HR_write_param(SI114_PARAM_PS1_ADC_MUX, 0x03);      // PS1 photodiode select
  HR_write_param(SI114_PARAM_PS2_ADC_MUX, 0x03);      // PS2 photodiode select
  HR_write_param(SI114_PARAM_PS3_ADC_MUX, 0x03);      // PS3 photodiode select

  HR_write_param(SI114_PARAM_PS_ADC_COUNTER, B01110000);    // B01110000 is default
  HR_write_reg(SI114_REG_COMMAND, B00001111);     // starts an autonomous read loop
//  Serial.println(HR_read_reg(SI114_REG_CHIP_STAT, 1), HEX);
  Serial.println("HR Sensor Initialized");
}

//--------------------------------------------------------------------------------------------
// Fram Code
/*
Data to be stored

variable      type       bytes    header    description
--------      -----      -----    ------    -----------
BPM_Average   float       4        0x08     Average Of User BPM during sampling
pso2          float       4        0x06     User SP02 
objt          float       4        0x05     Temperature of Skin
-X- diet          float       4        0x06     TMP006 Die Temp
gsr           float       4        0x07     GSR reading (figure out units, possibly mV)

Fram calculations:  
32768 bytes of storage / 4 bytes per data = 8192 data pieces
8192 data / 4 signals = 2048 data points per signal

At 5 min sleep between readings
24 hours * 60 minutes / 5 min = 288 cycles (for all 4 signals)
288 cycles * 4 signals per cycle * 4 bytes per signal = 4608 bytes used

At 2 min sleep between readings
24 hours * 60 minutes / 2 min = 720 cycles (for all 4 signals)
720 cycles * 4 signals per cycle * 4 bytes per signal = 11520 bytes used

*/

void FRAM_writeFloat(uint16_t framAddr, float value[])
{
  unsigned char *pointer;
  pointer = (unsigned char *) value;  
  Wire.beginTransmission(FRAMAddress);
  Wire.write(framAddr >> 8);
  Wire.write(framAddr & 0xFF);
  Wire.write(*(pointer)++);
  Wire.write(*(pointer)++);
  Wire.write(*(pointer)++);
  Wire.write(*(pointer));
  Wire.endTransmission();
}

void FRAM_writeInt(uint16_t framAddr, int value[])
{
  unsigned char *pointer;
  pointer = (unsigned char *) value;  
  Wire.beginTransmission(FRAMAddress);
  Wire.write(framAddr >> 8);
  Wire.write(framAddr & 0xFF);
  Wire.write(*(pointer)++);
  Wire.write(*(pointer));
  Wire.endTransmission();
}

float FRAM_readFloat(uint16_t framAddr)
{
  float f = 0.1;
  unsigned char *pc;
  pc = (unsigned char*)&f;
  
  Wire.beginTransmission(FRAMAddress);
  Wire.write(framAddr >> 8);
  Wire.write(framAddr & 0xFF);
  Wire.endTransmission();

  Wire.requestFrom(FRAMAddress, 4);
  pc[0] = Wire.read();
  pc[1] = Wire.read();
  pc[2] = Wire.read();
  pc[3] = Wire.read();
  
  *(unsigned int*)&f = (pc[3] << 24) | (pc[2] << 16) | (pc[1] << 8) | (pc[0] << 0);
  
//  result[0] = ((a[0]<<24) | (a[1]<<16) | (a[2]<<8) | (a[3]));
//  result[0] = ((a[3]<<24) | (a[2]<<16) | (a[1]<<8) | (a[0]));
  
  return f;
}

//--------------------------------------------------------------------------------------

void loop() {
//  Serial.println("Top of Main");

//    RFduino_resetPinWake(3); // reset state of pin that caused wakeup (Must do this)

  //Detect button Press
  if (RFduino_pinWoke(3)) {
    //RFduino_resetPinWake(3); // reset state of pin that caused wakeup (Must do this)
    Serial.println("Button Press!");
    state = BUTTON_PRESS;
    delay(2500);
    RFduino_resetPinWake(3); // reset state of pin that caused wakeup (Must do this)
  }
  
  if (state == RESET) {
    Serial.println("State: RESET");
//    if (prev_state == CONNECTED){
//      BLUE_LED_ON();
//      delay(1000);
//      LED_OFF();
//      delay(1000);
//    }
    
  }
  
  if (state == BUTTON_PRESS) { 
    Serial.println("State: BUTTON_PRESS");    
    LED_ON();
    delay(1000);
    LED_OFF();
    delay(1000);
    prev_state = BUTTON_PRESS;
    state = BLUETOOTH_ENABLE;
  }

  //Poll Sensors if time expired
  if (state == POLL_SENSORS) {
    Serial.println("State: POLL_SENSORS");
    //Do nothing if BLE is transmitting
    while (RFduinoBLE.radioActive)
      ;
      
    //Heart Rate Monitor Read
    Wire.beginTransmission(HRAddress); 
//    Serial.println("Heart Rate Read Attempt");
    int i, counts = 0;
    float BPM_Average = 0;
    float pso2_Average = 0;
    HR_EnableLEDs();
    for (i = 0; i < 350; i++) {
      readPulseSensor();
      if ((BPM > 50) && (BPM < 180)) {
//        Serial.print("\t BPM ");
//        Serial.print(BPM);
//        Serial.print("\t IR ");
//        Serial.print(IR_signalSize);
//        Serial.print("\t PSO2 ");
//        Serial.println(pso2, 3);
        BPM_Average += BPM;
        pso2_Average += pso2;
        counts++;
      }
    }
    HR_DisableLEDs();
    if (counts == 0) {
     counts = 1; 
    }
    BPM_Average = BPM_Average / counts;    // Dividing by 0 below happens when BPM outside of 50-180 BPM. This results in a nan - not a number.
//    Serial.print("\t Average BPM ");
//    Serial.println(BPM_Average);
    temp_data_float[0] = BPM_Average;
    Serial.print("Fram Address: "); Serial.print(fram_current_address);
    Serial.print(" BPM_Average written: "); Serial.println(temp_data_float[0]);
    FRAM_writeFloat(fram_current_address, temp_data_float);
    fram_current_address += 4;
    
    
    // PSO2 measurement
    pso2_Average = pso2_Average / counts;
    temp_data_float[0] = pso2_Average;
    Serial.print("Fram Address: "); Serial.print(fram_current_address);
    Serial.print(" pso2_Average written: "); Serial.println(temp_data_float[0]);
    FRAM_writeFloat(fram_current_address, temp_data_float);
    fram_current_address += 4;
    
    
    // Object temperature measurement     
    float objt = tmp006.readObjTempC();
    objt = objt * (9 / 5) + 32;
//    Serial.print("Object Temperature: "); Serial.print(objt);
//    Serial.println("*F");
    temp_data_float[0] = objt;
    Serial.print("Fram Address: "); Serial.print(fram_current_address);
    Serial.print(" objt written: "); Serial.println(temp_data_float[0]);
    FRAM_writeFloat(fram_current_address, temp_data_float);
    fram_current_address += 4;
    
    // TMP006 Die temperature measurement
//    fram_current_address += 4;
//    float diet = tmp006.readDieTempC();
//    diet = diet * (9 / 5) + 32;
////    Serial.print("Die Temperature: "); Serial.print(diet);
////    Serial.println("*F");
//    temp_data_float[0] = diet;
//    Serial.print("Fram Address: "); Serial.print(fram_current_address);
//    Serial.print(" diet written: "); Serial.println(temp_data_float[0]);
//    FRAM_writeFloat(fram_current_address, temp_data_float);
//    fram_current_address += 4;

    
    // Galvanic Skin Response
    float gsr = analogRead(4); // returns 0 to 1023
//    Serial.print("Galvanic Skin Response: "); Serial.println(gsr);
    temp_data_float[0] = gsr;
    Serial.print("Fram Address: "); Serial.print(fram_current_address);
    Serial.print(" gsr written: "); Serial.println(temp_data_float[0]);
    FRAM_writeFloat(fram_current_address, temp_data_float);
    fram_current_address += 4;
    
    
    //Next State
    prev_state = POLL_SENSORS;
    state = POLL_SENSORS;
  }

  if (state == BLUETOOTH_ENABLE) {
    Serial.println("State: BLUETOOTH_ENABLE");
    if (BLE_State == OFF) {
      Serial.println("Bluetooth Begin");
      RFduinoBLE.begin();
      delay(2000);
      Serial.println("Bluetooth On");
      BLE_State = ON;
      BLUE_LED_ON();
      delay(1000);
      LED_OFF();
      delay(1000);
      BLUE_LED_ON();
      delay(1000);
      LED_OFF();
      state = WAIT_TO_CONNECT; // Wait for a connection
    }
    else if (BLE_State == ON) {
      RFduinoBLE.end();
      Serial.println("Bluetooth ending");
      BLE_State = OFF;
      RED_LED_ON();
      delay(1000);
      LED_OFF();
      delay(1000);
      RED_LED_ON();
      delay(1000);
      LED_OFF();
      delay(1000);
      state = POLL_SENSORS; //
    } 
    prev_state = BLUETOOTH_ENABLE;
  }
  
  if (state == WAIT_TO_CONNECT) {
    Serial.println("State: WAIT_TO_CONNECT");
  }
  
  if (state == TRANSMIT_DATA) {
    Serial.println("State: TRANSMIT_DATA");
    /*
    Fram read order: BPM_Average, pso2_Average, objt, gsr
    */
    BLUE_LED_ON();
    
    while (RFduinoBLE.radioActive)
      ;
    
    // While all the data nas not yet been sent and BLE is still connected
    while ((fram_bluetooth_address <= fram_current_address) && (connectedBLE == true)){
      float temp_float;
      Serial.print("Fram Address: "); Serial.print(fram_bluetooth_address);
      temp_float = FRAM_readFloat(fram_bluetooth_address);
      Serial.print(" BPM_Average: "); Serial.println(temp_float);
      fram_bluetooth_address += 4;
      RFduinoBLE.sendByte(0x08); 
      delay(75);
      RFduinoBLE.sendFloat(temp_float); // BPM_Average
      delay(75);
      Serial.print("Fram Address: "); Serial.print(fram_bluetooth_address);
      temp_float = FRAM_readFloat(fram_bluetooth_address);
      Serial.print(" pso2_Average: "); Serial.println(temp_float);
      fram_bluetooth_address += 4;
      RFduinoBLE.sendByte(0x06); 
      delay(75);
      RFduinoBLE.sendFloat(temp_float); // objt
      delay(75);
      Serial.print("Fram Address: "); Serial.print(fram_bluetooth_address);
      temp_float = FRAM_readFloat(fram_bluetooth_address);
      Serial.print(" objt: "); Serial.println(temp_float);
      fram_bluetooth_address += 4;
      RFduinoBLE.sendByte(0x05); 
      delay(75);
      RFduinoBLE.sendFloat(temp_float); // objt
      delay(75);
//      Serial.print("Fram Address: "); Serial.print(fram_bluetooth_address);
//      temp_float = FRAM_readFloat(fram_bluetooth_address);
//      Serial.print(" diet: "); Serial.println(temp_float);
//      fram_bluetooth_address += 4;
//      RFduinoBLE.sendByte(0x06); 
//      delay(75);
//      RFduinoBLE.sendFloat(temp_float); // diet
//      delay(75);
      Serial.print("Fram Address: "); Serial.print(fram_bluetooth_address);
      temp_float = FRAM_readFloat(fram_bluetooth_address);
      Serial.print(" gsr: "); Serial.println(temp_float);
      fram_bluetooth_address += 4;
      RFduinoBLE.sendByte(0x07); 
      delay(75);
      RFduinoBLE.sendFloat(temp_float); // gsr
      delay(75);
    }
    
    GREEN_LED_ON();
    prev_state = TRANSMIT_DATA;
    state = RESET; // To turn off BLE advertising and begin polling sensors
  }
  
  else if (state == FRAM_TESTING) {
    Serial.println("State: FRAM_TESTING");
    Serial.print("Fram Address: "); Serial.println(fram_current_address);
    Serial.print("Write Data: "); Serial.println(temp_data_float[0]);
    FRAM_writeFloat(fram_current_address, temp_data_float);
    fram_current_address += 4;
    temp_data_float[0] += 1.1;
    
    Serial.print("Read Fram at Address: "); Serial.println(fram_bluetooth_address);
    float temp = FRAM_readFloat(fram_bluetooth_address);
    Serial.print("Value at address: "); Serial.println(temp);
    fram_bluetooth_address += 4;
  }

  //Sleep for 2 minutes or until interrupt
  if (state != TRANSMIT_DATA) {    //TRANSMIT_DATA //WAIT_TO_CONNECT
    RFduino_ULPDelay(MINUTES(2)); 
//    RFduino_ULPDelay(SECONDS(10));

  }

}
