/*!
 * ************************************************************************************************************************
 *  Version 2.1 @ 31.12.2017 initial work by hdgucken(Thomas, thanks for sharing) + juergs (Juergen)
 * ************************************************************************************************************************
 * @file bme680_cc_rfm69cw_sensor.ino
 * @location: C:\Users\jschw\Documents\Arduino\_BME680_CO2\BME680_cc_sensor_v1_5_V2_D1\bme680_cc_sensor
 * @brief
 * Example for using of BSEC library in a fixed configuration with the BME680 sensor.
 * This works by running an endless loop in the bsec_iot_loop() function.
 * Presets for ESP8266:
 *	C:\Users\<user>\Documents\Arduino\BSEC_1.4.5.1_Generic_Release_20171214\BSEC_1.4.5.1_Generic_Release_20171214\algo\bin\esp8266\libalgobsec.a
 *  into folder:  
 * 	C:\Users\<user>\AppData\Local\Arduino15\packages\esp8266\hardware\esp8266\2.4.0-rc1\tools\sdk\lib
 *  See BSEC doku, 
 *	Download: https://www.bosch-sensortec.com/bst/products/all_products/bsec
 *  ESP8266: "Wemos® D1 Esp-Wroom-02 Motherboard ESP8266 Mini-WiFi NodeMCU Module ESP8266+18650 Battery+0.96 OLED" from ebay 
 *  
 * *********************************************************************************************************************
 *
 * @addtogroup bsec_examples BSEC Examples
 * @brief BSEC usage examples
 * @{*/
/*********************************************************************************************************************

  Hardware setup:                                                   BME680              RFM69CW
                         NodeMCU                                   +------+            +-------+
                         +--\/--+                            +3.3V | 0x77 |      +3,3V |       | GND
                VCC 3,3V |      | GND                      <-> SDA |      |   --> MOSI |    NSS| RFM_NSS <--
    int.LED (D0) GPIO 16 |      | GPIO 1  (D10)            --> SCL |      |   <-- MISO |   DIO0| RFM_IRQ -->
    <-- SCL (D1)  GPIO 5 |      | GPIO 3  (D9)                 GND |      |   --> SCK  |       |
           RESET     RST |      | GPIO 15 (D8) RFM_NSS -->         +------+            +-------+
    <-> SDA (D2)  GPIO 4 |      | GPIO 13 (D7) MOSI -->             BH1750            SH1106/SH1306 - OLED
            (D3)  GPIO 0 |      | GPIO 12 (D6) MISO <--            +------+            +-------+
--> RFM_IRQ (D4)  GPIO 2 |      | GPIO 14 (D5) SCK  -->      +3.3V | 0x23 |      +3.3V | 0x3C  |
                         +------+                              GND |      |        GND |       |
                                                           --> SCL |      |    --> SCL |       |
                                                           <-> SDA |      |    <-> SDA |       |
                                                                   +------+            +-------+                      
  Additional infos:                                                                 
    http://www.watterott.com/de/BME680-Breakout                                                                   
    Code: 
    https://github.com/watterott/BME680-Breakout
    Schmatic & Layout:
    https://github.com/watterott/BME680-Breakout/tree/master/hardware

  Hardware:
    Wemos ESP-12F Hauptplatine WiFi Modul ESP8266 18650 Nodemcu Batterie+0.96” OLED  (Esp8266 with keypad, ebay) 
    Micro USB Development Board 0.96" OLED NODEMCU Wemos Wifi ESP8266 ESP-12F CP2102 (Esp8266 with OLED only, ebay)
    WeMos D1 MINI - ESP8266 ESP12 NodeMcu Dev-Kit WiFi Modul Board Arduino (ESP8266, see code due other init settings for oled, ebay) 
    General Infos:    http://esp8266.github.io/Arduino/versions/2.3.0/doc/reference.html

  Remarks:
  https://stackoverflow.com/questions/34497758/what-is-the-secret-of-the-arduino-yieldfunction/34498165

  Sample for "yield" in due to possible WDT-reset after 3.6 sec:  
    pinMode(12, INPUT_PULLUP); // Set pin 12 as an input w/ pull-up
    while (digitalRead(12) == HIGH) // While pin 12 is HIGH (not activated)
    yield(); // Do (almost) nothing -- yield to allow ESP8266 background functions
    Serial.println("Button is pressed!"); // Print button pressed message.

  RFM69CW - Custom Sensor Protokol:
  =================================

      OK CC  ID  T1 T2  HH  P1 P2  I1 I2  L1 L2  UB  G1 G2 G3 CRC     

      100er + 10er bedeutet linkes Nibble 100er, rechtes Nibble 10er usw. (BCD Code) !
      
      OK CC - fest, für CustomSensor Protokoll
      ID - Sensor ID (0-FF)
      T1 - (Temp + 40 * 10): 100er
      T2 - (Temp + 40 * 10): 10er + 1er
      HH - humidity 1-99
      P1 - pressure = 1000er + 100er                                   
      P2 - pressure = 10er + 1er
      I1 - IAQ = 100er
      I2 - IAQ = 10er + 1er
      L1 - LightLevel = 1000er + 100er
      L2 - LightLevel = 10er + 1er
      UB - BatteryVoltage * 10 = (0-255; 3,3V = 33)
      G1 - gas resistance = 100.000er + 10.000er
      G2 - gas resistance = 1.000er + 100er
      G3 - gas resistance = 10er + 1er
      CRC - crc byte

  OLED-LIB: SSD1306ASCII:
  =======================
  See details on fonts etc. on  "MainPage\SSD1306mainpage.h"
  https://github.com/greiman/SSD1306Ascii
  Giving "println()" for comfort.
  Details on documentation "..SSD1306Ascii-master/SSD1306Ascii/html/index.html"

  ReadVCC:
  ========
    //at the beginning of sketch
    ADC_MODE(ADC_VCC); //vcc read
    ...
    //in loop
    float vdd = ESP.getVcc() / 1000.0;


  D1 mini-Board:
  ==============
    https://wiki.wemos.cc/doku.php
    Schematic: https://wiki.wemos.cc/_media/products:d1:mini_new_v2_2_0.pdf
  
*/
/*********************************************************************************************************************** 
 * Wireless sensor for measuring temperature, humidity, pressure, lightlevel and airquality.
 * Hardware: NodeMCU V1.0, RFM69CW, BME680, BH1750 and 1.3 inch 128x64 LCD.
 * Protokoll is CustomSensor.
 * Frequency is 868.3 MHz (or 433 MHz / 915 MHz based on RFM69CW model).
 * V1.3  10.11.2017, T.Hirte
 * V1.4  include BH1750 light sensor, 17.11.2017, T.Hirte
 * V1.5  umgestellt auf Bosch Sensortec Software vom 17.11.2017, 01.12.2017, T.Hirte
 * V1.6  adaptiert für Wemos D1 with integrated OLED + 18650 and Wemos D1 with separate (white) OLED 128x64.  (see board specific wire-settings in code!) 
 * V2.0  OLED Lib set to "SSD1306ASCII", because ADAFRUIT-Lib doesnt function with text, but grafics only. (;-(  
 ***********************************************************************************************************************/
#define HAS_OLED_DISPLAY      true
#define HAS_LIGHTSENSOR       false
#define HAS_RFM_MODULE        true 
#define IS_RFM_MANDATORY      false
#define IS_WROOM2_BOARD       true
#define IS_ADFRUIT_LIB        false 
#define ENABLE_SERIAL_OUTPUT  true 


// ------------------begin ESP8266'centric----------------------------------
// see: https://www.hackster.io/rayburne/esp8266-turn-off-wifi-reduce-current-big-time-1df8ae
//      http://bbs.espressif.com/viewtopic.php?t=836
#define FREQUENCY             80                  // valid 80, 160
//
#include "ESP8266WiFi.h"
extern "C" {
#include "user_interface.h"
}
// ------------------end ESP8266'centric------------------------------------


/***********************************************************************************************************************/
/* header files */
/***********************************************************************************************************************/
#include <Wire.h>
#include "bsec_integration.h"
#include "ADC_Helper.h"       // workaround to read VDD.
#include "movingAvg.h"        // uint32
#include "movingAvgFloat.h"   // float

#if (IS_ADFRUIT_LIB ) 
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
#else
  #include <SSD1306Ascii.h>
  #include <SSD1306AsciiWire.h>
#endif 

#if (HAS_RFM_MODULE) 
  #include "RFMxx.h"
  #include "SensorBase.h" //calculate crc byte
  SensorBase                    crc8;
#endif 

#if (HAS_LIGHTSENSOR)
  //#include <AS_BH1750.h>
#endif 

/********************************/
/*     variables                */
/********************************/

//--- OLED 128x64 SSD1306.
#define       I2C_ADDRESS_OLED            0x3C      //--- basic OLED-I2C-address

/* Sensor Config */
#define       VERSION               "V2.0"
#define       NODEID                11        //every node needs his own unique ID 
#define       BME680_I2C_ADDR       0x77     //BlueDot has 0x77, standard address is 0x76 

bool          DEBUG               = true;   //activate debug mode

unsigned long DATA_RATE           = 17241ul; //default data rate (for transmit on RFM69)
unsigned long INITIAL_FREQ        = 868300;  //default frequency in kHz (5 kHz steps, 860480 ... 879515) 

float         TEMP_CORR           = 1.6f;    //adjust temperature, 2.4f => -2.4 degrees !

uint8_t       loop_count_lim      = 20;      //time to wait between data transmissions (20: 20 * 3sec = 60sec)
uint8_t       loop_counter;

//--- sendarray, encode Payload CustomSensor 
byte          bytes[17];

#define LEDpin    D0       //[BUILTIN_LED]=D4 is blue LED on ESP12, D0 is green LED on Board, auto or set pin of your choice

#if (HAS_RFM_MODULE)
    /* RFM69, PIN-config SPI (GPIO XX): */
    #define RFM_SS           D8 //(D8)   SS  -> RFM69 (NSS)
    #define RFM_MISO         D6 //(D6) MISO  <- RFM69 (MOSI) //only used by soft spi 
    #define RFM_MOSI         D7 //(D7) MOSI  -> RFM69 (MISO) //only used by soft spi
    #define RFM_SCK          D5 //(D5)  SCK  -> RFM69 (SCK)  //only used by soft spi
    #define RFM_IRQ          D0 //(D4)  IRQ  <- RFM69 (DIO0)    
    //--- create the rfm instance
    //RFMxx                    rfm(RFM_MOSI, RFM_MISO, RFM_SCK, RFM_SS, RFM_IRQ, NULL);
    RFMxx                    rfm(RFM_MOSI, RFM_MISO, RFM_SCK, RFM_SS);
    //RFMxx                  rfm; 
#endif 

#if HAS_LIGHTSENSOR
    /* Lichtsensor  */
    #define BH1750_ADR       0x23  //set I2C adress 0x23 or 0x5C    
    AS_BH1750                bh1750(BH1750_ADR);
#endif

  /* has OLED display */
bool    OLED =               HAS_OLED_DISPLAY;  //activate the OLED display (SH1106 or SSD1306)

#if (IS_ADFRUIT_LIB)
  Adafruit_SSD1306           display;
#else
  SSD1306AsciiWire           display;
#endif 

// --- lowpass-filter over 6 readings 
movingAvg IAQ_QUER; 

/* ****************************** */
/*            functions           */
/* ****************************** */

//---------------------------------------------------------------------
void SetDebugMode(boolean mode)
{
  DEBUG = mode;
  #if (HAS_RFM_MODULE)
    rfm.SetDebugMode(mode);
    crc8.SetDebugMode(mode);
  #endif   
}
//---------------------------------------------------------------------
// blink led
static void blink (byte pin, byte n = 3, int del = 50)
{
  for (byte i = 0; i < n; ++i) 
  {
    digitalWrite(pin, LOW);
    delay(del);
    digitalWrite(pin, HIGH);
    delay(del);
  }
}
//---------------------------------------------------------------------
/*!
 * @brief           Write operation in either Wire or SPI
 * param[in]        dev_addr        Wire or SPI device address
 * param[in]        reg_addr        register address
 * param[in]        reg_data_ptr    pointer to the data to be written
 * param[in]        data_len        number of bytes to be written
 * @return          result of the bus communication function
 */
int8_t bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr, uint16_t data_len)
{
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);    /* Set register address to start writing to */
    /* Write the data */
    for (int index = 0; index < data_len; index++) 
    {
        Wire.write(reg_data_ptr[index]);
    }
    return (int8_t)Wire.endTransmission();
}
//---------------------------------------------------------------------
/*!
 * @brief           Read operation in either Wire or SPI
 * param[in]        dev_addr        Wire or SPI device address
 * param[in]        reg_addr        register address
 * param[out]       reg_data_ptr    pointer to the memory to be used to store the read data
 * param[in]        data_len        number of bytes to be read
 * @return          result of the bus communication function
 */
int8_t bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr, uint16_t data_len)
{
    int8_t comResult = 0;
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);                    /* Set register address to start reading from */
    comResult = Wire.endTransmission();
    delayMicroseconds(150);                 /* Precautionary response delay */
    Wire.requestFrom(dev_addr, (uint8_t)data_len);    /* Request data */
    int index = 0;
    while (Wire.available())  /* The slave device may send less than requested (burst read) */
    {
        reg_data_ptr[index] = Wire.read();
        index++;
    }
    return comResult;
}
//---------------------------------------------------------------------
/*!
 * @brief           System specific implementation of sleep function
 * @param[in]       t_ms    time in milliseconds
 * @return          none
 */
void sleep(uint32_t t_ms)
{
    delay(t_ms);
    //yield();  = delay(0); 
}
//---------------------------------------------------------------------
/*!
 * @brief           Capture the system time in microseconds
 * @return          system_current_time    current system timestamp in microseconds
 */
int64_t get_timestamp_us()
{
    return ( (int64_t) millis() * 1000 );
}
//---------------------------------------------------------------------
int64_t print_timestamp()
{
    return ( (int64_t) get_timestamp_us() / 1e6 );   
}
//---------------------------------------------------------------------
int64_t serial_timestamp()
{
    //--- a jumper to GND on D10 does the job, when needed! 
   /* if (digitalRead(PIN_ENABLE_TIMESTAMP_IN_OUTPUT) == LOW)
    {*/
        Serial.print("[");
        Serial.print(get_timestamp_us() / 1e6);
        Serial.print("] ");
    //}
}
//---------------------------------------------------------------------
    /*!
 * @brief           Load previous library state from non-volatile memory
 * @param[in,out]   state_buffer    buffer to hold the loaded state string
 * @param[in]       n_buffer        size of the allocated state buffer
 * @return          number of bytes copied to state_buffer
 */
uint32_t state_load(uint8_t *state_buffer, uint32_t n_buffer)
{
    // ...
    // Load a previous library state from non-volatile memory, if available.
    //
    // Return zero if loading was unsuccessful or no state was available, 
    // otherwise return length of loaded state string.
    // ...
    return 0;
}
//---------------------------------------------------------------------
/*!
 * @brief           Save library state to non-volatile memory
 * @param[in]       state_buffer    buffer holding the state to be stored
 * @param[in]       length          length of the state string to be stored
 * @return          none
 */
void state_save(const uint8_t *state_buffer, uint32_t length)
{
    // ...
    // Save the string some form of non-volatile memory, if possible.
    // ...
}
//---------------------------------------------------------------------
/*!
 * @brief           Load library config from non-volatile memory
 * @param[in,out]   config_buffer    buffer to hold the loaded state string
 * @param[in]       n_buffer        size of the allocated state buffer
 * @return          number of bytes copied to config_buffer
 */
uint32_t config_load(uint8_t *config_buffer, uint32_t n_buffer)
{
    // ...
    // Load a library config from non-volatile memory, if available.
    //
    // Return zero if loading was unsuccessful or no config was available, 
    // otherwise return length of loaded config string.
    // ...
    return 0;
}
//---------------------------------------------------------------------
/*!
 * @brief           Handling of the ready outputs
 * @param[in]       timestamp       time in nanoseconds
 * @param[in]       iaq             IAQ signal
 * @param[in]       iaq_accuracy    accuracy of IAQ signal
 * @param[in]       temperature     temperature signal
 * @param[in]       humidity        humidity signal
 * @param[in]       pressure        pressure signal
 * @param[in]       raw_temperature raw temperature signal
 * @param[in]       raw_humidity    raw humidity signal
 * @param[in]       gas             raw gas sensor signal
 * @param[in]       bsec_status     value returned by the bsec_do_steps() call
 * @return          none
 */
void output_ready(int64_t timestamp, float iaq, uint8_t iaq_accuracy, float temperature, float humidity,
                  float pressure, float raw_temperature, float raw_humidity, float gas, bsec_library_return_t bsec_status)
{
      /* get battery voltage */
      //int vcc = analogRead(A0);
      float vdd = ESP.getVcc() / 1000;

      #if (HAS_LIGHTSENSOR)
        /* get light level */
        float lux = bh1750.readLightLevel();
      #else
        float lux = 0.0; 
      #endif 

      //--- calculate IAQ-Mean over 6 readings 
      IAQ_QUER.reading((uint8_t)iaq);
      
      if (OLED) 
      {
        display.setCursor(0,0);
        display.clear();
      }

      //yield(); 

      #if (ENABLE_SERIAL_OUTPUT)
        /* Serial Output Measurements */ 
        Serial.print("[");
        Serial.print(timestamp/1e6,0);
        Serial.print("] P: ");
        Serial.print(pressure/100);
        Serial.print("| T: ");
        Serial.print(temperature);
        Serial.print("| rH: ");
        Serial.print(humidity);
        Serial.print("| IAQ: ");
        Serial.print((uint8_t)iaq);Serial.print("  ");Serial.print((uint8_t)IAQ_QUER.getAvg());
        Serial.print(" (");
        Serial.print(iaq_accuracy);
        Serial.print(")");
        Serial.print("| Gas: ");
        Serial.print((uint64_t)gas/1000.f);
        Serial.print("| UBat: ");
        //Serial.print((float) vcc/100);
        Serial.print((float) (vdd * 1.0891));
        Serial.print("V");
        Serial.print("| Light: ");
        Serial.print((uint8_t)lux);
        Serial.println("lx");
      #endif 
      
      /* on OLED Display */

     // IAQ_QUER.reading((uint8_t)iaq);
      
      if (OLED)
      {
        display.set1X();
        display.setCursor(0,0);
        display.print("TEMP:  \t"); display.print(temperature,1);       display.println(" Grad"); 
        display.print("HUM:   \t"); display.print((uint8_t)humidity);   display.println(" %"); 
        display.print("PRESS: \t"); display.print((int)pressure / 100); display.println(" mBar"); 
        display.print("GAS:   \t"); display.print(gas/1E3);             display.println(" KOhm"); 
        display.print("BAT:   \t"); display.print(vdd*1.0891F);         display.print(" V"); 
        display.print("  A: \t");   display.println(iaq_accuracy);
        //display.println();
        display.print("IAQ:   \t"); display.println((uint8_t)iaq);
        display.set2X();
        display.print("IAQ_:  ");   display.println((uint8_t)IAQ_QUER.getAvg());
                
        //display.set1X();
        //display.print("ACC: \t"); display.println(iaq_accuracy);        
      }



      loop_counter += 1; 
      if ( loop_counter >= loop_count_lim ) 
      {  
        //send every (loop_count_lim * 3) seconds 
        loop_counter = 0;
        
        //--- sendarray, encode Payload CustomSensor 
        //byte bytes[17];    // jetzt global, memory/heap-overflow?

        //--- clear buffer
        for (uint8_t i = 0; i < sizeof(bytes); i++) { bytes[i] = 0; }
        
        //--- fix  "CC" FOR CUSTOM SENSOR 
        bytes[0] = 0xCC;
        
        /* NODE-ID */
        bytes[1] = NODEID;
        
        /* NR. OF DATABYTES */
        bytes[2] = sizeof(bytes) - 4;
        
        /*** TEMPERATURE ***/
        int temp_int = (temperature + 40.05) * 10.0; //+40 to get positive values, +0.05 to round, *10 for 3 digits
        bytes[3]  = temp_int / 100;                  //100's
        bytes[4]  = ((temp_int % 100) / 10) << 4;    //is ok, values are in range of 150 - 900 ;o)
        bytes[4] |= temp_int % 10;
        
        /*** HUMIDITY ***/
        uint8_t     hum_int = humidity + 0.5;            //rounded
        bytes[5] =  hum_int;
        
        /*** PRESSURE ***/
        int pre_int = (pressure / 100.0) + 0.5;      //pressure rounded in hPa without komma
        
        if ( pre_int > 999 && pre_int < 10000 ) { 
          bytes[6]  = (pre_int / 1000) << 4;         //1000's 
          bytes[6] |= (pre_int / 100) % 10;          //100's 
          bytes[7]  = ((pre_int % 100) / 10) << 4;   //10's
          bytes[7] |= pre_int % 10;                  //1's
        }
        else if ( pre_int > 99 && pre_int < 1000 ) {
          bytes[6]  = pre_int / 100;                 //100's
          bytes[7]  = ((pre_int % 100) / 10) << 4;   //10's
          bytes[7] |= pre_int % 10;                  //1's
        }
        else if ( pre_int > 9 && pre_int < 100 ) {
          bytes[7]  = (pre_int / 10) << 4;           //10's
          bytes[7] |= pre_int % 10;                  //1's
        }
        else if ( pre_int < 10 ) {
          bytes[7] = pre_int;                        //1's
        }
         
        /*** IAQ ***/
        int iaq_int = iaq + 0.5;                     //rounded

        if ( iaq_int > 99 && iaq_int < 501 ) {       // because iaq max. is 500
          bytes[8]  = iaq_int / 100;                 //100's
          bytes[9]  = ((iaq_int % 100) / 10) << 4;   //10's
          bytes[9] |= iaq_int % 10;                  //1's
        }
        else if ( iaq_int > 9 && iaq_int < 100 ) {
          bytes[9]  = (iaq_int / 10) << 4;           //10's
          bytes[9] |= iaq_int % 10;                  //1's
        }
        else if ( iaq_int < 10 ) {
          bytes[9] = iaq_int;                        //1's
        }
        
        /*** LIGHTLEVEL ***/
        int lux_int = lux + 0.5;                     //rounded

        if ( lux_int > 999 && lux_int < 10000 ) { 
          bytes[10]  = (lux_int / 1000) << 4;        //1000's 
          bytes[10] |= (lux_int / 100) % 10;         //100's 
          bytes[11]  = ((lux_int % 100) / 10) << 4;  //10's
          bytes[11] |= lux_int % 10;                 //1's
        }
        else if ( lux_int > 99 && lux_int < 1000 ) {
          bytes[10]  = lux_int / 100;                //100's
          bytes[11]  = ((lux_int % 100) / 10) << 4;  //10's
          bytes[11] |= lux_int % 10;                 //1's
        }
        else if ( lux_int > 9 && lux_int < 100 ) {
          bytes[11]  = (lux_int / 10) << 4;          //10's
          bytes[11] |= lux_int % 10;                 //1's
        }
        else if (lux_int < 10) {
          bytes[11] = lux_int;                       //1's
        }
                
        /*** BATTERY VOLTAGE ***/
        bytes[12] = ((uint8_t) vdd + 5) * 10;                  //was vcc = 330mV at 3,3V
        
        /*** GAS RESISTANCE ***/
        uint32_t gas_int = gas;
        
        if ( gas_int > 99999 && gas_int < 1000000 ) 
        {
          bytes[13]  = (gas_int / 100000) << 4;          //100000's 
          bytes[13] |= (gas_int / 10000) % 10;           //10000's
          bytes[14]  = ((gas_int / 1000) % 10) << 4;     //1000's
          bytes[14] |= (gas_int / 100) % 10;             //100's
          bytes[15]  = (gas_int % 10) << 4;              //10's
          bytes[15] |= (gas_int * 10) % 10;              //1's
        }        
        else if ( gas_int > 9999 && gas_int < 100000 ) {
          bytes[13]  = gas_int / 10000;                  //10000's
          bytes[14]  = ((gas_int / 1000) % 10) << 4;     //1000's
          bytes[14] |= (gas_int / 100) % 10;             //100's
          bytes[15]  = ((gas_int / 10) % 10) << 4;       //10's
          bytes[15] |= gas_int % 10;                     //1's
        }
        else if ( gas_int > 999 && gas_int < 10000 ) {
          bytes[14]  = (gas_int / 1000) << 4;            //1000's
          bytes[14] |= (gas_int / 100) % 10;             //100's
          bytes[15]  = ((gas_int / 10) % 10) << 4;       //10's
          bytes[15] |= gas_int % 10;                     //1's
        }
        else if ( gas_int > 99 && gas_int < 1000 ) {
          bytes[14]  = gas_int / 100;                    //100's
          bytes[15]  = ((gas_int / 10) % 10) << 4;       //10's
          bytes[15] |= gas_int % 10;                     //1's
        }
        else if ( gas_int > 9 && gas_int < 100 ) {
          bytes[15]  = (gas_int / 10) << 4;              //10's
          bytes[15] |= gas_int % 10;                     //1's
        }
        else if ( gas_int < 10 ) {
          bytes[15] = gas_int;                           //1's
        }
        
      #if (HAS_RFM_MODULE) 
        /* CRC BYTE */
        uint8_t crc = crc8.CalculateCRC(bytes, sizeof(bytes) - 1);
        bytes[16] = crc; // CRC over payload bytes 0-15        
        rfm.SendArray( (uint8_t *)bytes, sizeof(bytes) );
        rfm.PowerDown();
      #endif 

      // ------------------begin ESP8266'centric----------------------------------
      // patch the watchdog timer
      wdt_reset();
      // ------------------end ESP8266'centric------------------------------------

      if (DEBUG) 
      {
        Serial.print(sizeof(bytes)); Serial.print(" bytes"); Serial.println(" sent.");        
        Serial.print("heapsize: "); Serial.println( ESP.getFreeHeap() );
        Serial.println(); 
      }
      
      blink(LEDpin,1,25);
    }
}
//---------------------------------------------------------------------
/*!
 * @brief       Main function which configures BSEC library and then reads and processes the data from sensor based
 *              on timer ticks
 *
 * @return      result of the processing
 */
void setup()
{
    /* Initializations */
    // ------------------begin ESP8266'centric----------------------------------
    WiFi.forceSleepBegin();                  // turn off ESP8266 RF
    delay(1);                                // give RF section time to shutdown
    system_update_cpu_freq(FREQUENCY);
    // ------------------end ESP8266'centric------------------------------------

    return_values_init  ret;     
    
    //ADC_MODE(ADC_VCC); //vcc read

    SetDebugMode(DEBUG);
    
    Serial.begin(115200);
    sleep(3000); //some time to open the terminalprogram ;o)
    
    Serial.println("");
    Serial.print("BME680 wireless sensor ");
    Serial.println(VERSION);
    Serial.println("Protocol: CustomSensor");
    Serial.print("NODE-ID : ");
    Serial.println(NODEID);
        
    /* init I2C */
    #if (IS_WROOM2_BOARD)  
      //--- Wroom2-Board needs this (false) variant, because switched pins (!).
      //--- therefore standard constructor not succeed! 
      Wire.begin(D1,D2);         
    #else
      Wire.begin();         
    #endif
    
    Wire.setClock(400000); 
    
    if (OLED) 
    {
      /* init OLED */
      display.begin(&Adafruit128x64, I2C_ADDRESS_OLED);      
      display.setFont(Adafruit5x7);      
      
      //display.setTextColor(WHITE); // used white only display 
      display.setCursor(5,4);      
      display.clear();

      /* startscreen */
      display.set2X();
      display.println("  BME680");      
      display.set1X(); 
      display.println("");
      display.print("Wireless Sensor ");
      display.println(VERSION);
      display.println("CustomSensor Protocol");
      display.print("Node-ID: ");
      display.println(NODEID);
      
      sleep(5000);

      display.setContrast(0x96);              
      display.setCursor(0,0);      
      display.clear();
    }
    
    #if(HAS_RFM_MODULE) 
      /* --- RFM69CW init --- */
      //SPI.begin();
      //rfm.Begin();
      
      if ( rfm.IsConnected() ) 
      {
        if (OLED) 
        {
          display.println("RFM69 gefunden.");
          delay(1000);
          //display.display();
        }
        Serial.println("RFM69 found.");
              
        rfm.InitializeLaCrosse();
        if (DEBUG) Serial.println("Init LaCrosse ok.");
        
        rfm.SetFrequency(INITIAL_FREQ);
        float init_freq   = float(INITIAL_FREQ);
        display.print("Frequenz: ");
        display.print(init_freq/1000,3);
        display.println(" MHz");
        if (DEBUG) 
        {
          Serial.print("Set frequency to ");
          Serial.print(init_freq/1000,3);
          Serial.println(" MHz.");
        }
        
        rfm.SetDataRate(DATA_RATE);        
        
        if (OLED) 
        {
          display.print("Baudrate: ");
          display.print(DATA_RATE);
          display.println(" Baud");
        }
        if (DEBUG) 
        {
          Serial.print("Set datarate to ");
          Serial.print(DATA_RATE);
          Serial.println(" bps.");
        }
        
        rfm.PowerDown(); // sleep to save power

        Serial.println("RFM69 initialized.");
        if (OLED) 
        {
          display.println("RFM69 bereit.");
          delay(1000); 
          //display.println("");
          //display.display();
        }
      }
      else 
      {
        if (OLED) 
        {
          display.println("RFM69 Fehler, Stop!");
          //display.display();
        }
        Serial.println("RFM69 failed, stop!");
        blink(LEDpin, 6, 500);
        
        #if (IS_RFM_MANDATORY)
          for (;;); // spin fo rever
        #endif 
      }
    #endif 
    
    /* --- BME680 init --- */
    Serial.print("BME680: try init on 0x");
    Serial.print(BME680_I2C_ADDR, HEX);
    Serial.print(" ... ");
    if (OLED) 
    {
      display.print("BME680 init auf 0x");
      display.println(BME680_I2C_ADDR, HEX);
      //display.display();
      delay(1000); 
    }
    
    /* Call to the function which initializes the BSEC library 
     * Switch on low-power mode and provide no temperature offset */
    ret = bsec_iot_init(BSEC_SAMPLE_RATE_LP, TEMP_CORR, bus_write, bus_read, sleep, state_load, config_load, BME680_I2C_ADDR);
    
    if (ret.bme680_status)
    {
        /* Could not intialize BME680 */
        Serial.println("Error while initializing, wrong I2C addr ?");
        if (OLED) {
          display.println("BME680 init. Fehler!");
          display.println("Falsche I2C Adresse?");
          delay(500); 
        }
        return;
    }
    else if (ret.bsec_status)
    {
        /* Could not intialize BSEC library */
        Serial.println("Error while initializing BSEC library !");
        if (OLED) {
          display.println("BSEC init. Fehler !");
          delay(500); 
          // display.display();
        }
        return;
    }
    
    Serial.println("done.");
    if (OLED) 
    {
      display.println("bereit");
      delay(5000); 
    }

    #if HAS_LIGHTSENSOR 
      /* --- BH1750 init --- */
      Serial.print("BH1750 init ... ");
      if (OLED) 
      {
        display.print("BH1750 init...");
        // display.display();
      }
      
      /* for normal sensor resolution (1 lx resolution, 0-65535 lx, 120ms, no PowerDown) use: bh1750.begin(RESOLUTION_NORMAL, false); */
      if ( bh1750.begin() ) {
        Serial.println("done.");
        Serial.println("Ready, start measuring ...");
        Serial.println("");
        if (OLED) 
        {
          display.println("bereit");
          sleep(6000);
          display.print("Starte Messungen ...");
          // display.display();
          sleep(2000);
        }
      } else 
      {
        if (OLED) 
        {
          display.println("");
          display.print("BH1750 Fehler, Stop!");
          // display.display();
        }
        Serial.print("BH1750 failed, stop!");
        blink(LEDpin, 6, 500);
        for (;;); // spin fo rever
      }
    #endif 
    
    loop_counter = loop_count_lim; //to send data at startup
    
    /* Call to endless loop function which reads and processes data based on sensor settings */
    /* State is saved every 10.000 samples, which means every 10.000 * 3 secs = 500 minutes  */
    bsec_iot_loop(sleep, get_timestamp_us, output_ready, state_save, 10000);
}
//---------------------------------------------------------------------
void loop()
{
  // we are looping in BSEC ! 
}
//---------------------------------------------------------------------
//---------------------------------------------------------------------
/*! @}*/
