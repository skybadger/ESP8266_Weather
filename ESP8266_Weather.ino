/*
 Program to talk to weather sensors and send output to MQTT topic /skybadger/device/temperature
 Supports REST web interface on port 80 returning application/json string
 
To do:


Done: 
fix all x.begin errors to represent their status as to whether present on the bus . 
Replace encoder with ADC i2c device to detect wind bearing

Sensors added are:
ADC AD1152 speed gauge              (1 pin) speed indicator is 1.429 mph per ticks per sec. 
rain gauge sensor - pulse counter.  (1 pin)
wind direction gauge uses ADC       ( share 2 i2c pins) or use built-in on pin 11 pin?
Use HTU21D for humidity and temp.   ( share 2 i2c pins)
Use BMP280 for pressure and temp    ( share 2 i2c pins) 

 Layout:
 https://learn.adafruit.com/adafruit-huzzah-esp8266-breakout/pinouts
 Note: GPIO16 cannot have interupts attached to it.. https://espwifi.readthedocs.io/en/latest/reference.html
 GPIO 12 to Rain gauge pulse data in (12v) divided down to 3v3
 GPIO 14 to Wind cup anemometer pulse data in (3v3)
 A0 to 1v max range from Wind Direction sensor
 GPIO 4,2 to SDA with 3K3 pullup to 3v3
 GPIO 5,0 to SCL with 3K3 pullup to 3v3
 All 3.3v logic. 
 */

#include "DebugSerial.h"
#include <Esp.h>          //used for restart
#include <esp8266_peri.h> //register map and access
#include <ESP8266WiFi.h>
#include <PubSubClient.h> //https://pubsubclient.knolleary.net/api.html
#include <EEPROM.h>
#include <Wire.h>         //https://playground.arduino.cc/Main/WireLibraryDetailedReference
#include <Time.h>         //Look at https://github.com/PaulStoffregen/Time for a more useful internal timebase library
#include <WiFiUdp.h>      //for debug handler and ALPACA discovery - that would imply observing conditions support. 
#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>
#include <ArduinoJson.h>  //https://arduinojson.org/v5/api/
#include <Math.h>         //Used for PI.
#include <GDBStub.h> //Debugging stub for GDB
#define MAX_TIME_INACTIVE 0 //to turn off the de-activation of a telnet session
#include "RemoteDebug.h"  //https://github.com/JoaoLopesF/RemoteDebug

//Time
#include <time.h>
#include <sys/time.h>
#include <coredecls.h>
#define TZ              0       // (utc+) TZ in hours
#define DST_MN          60      // use 60mn for summer time in some countries
#define TZ_MN           ((TZ)*60)
#define TZ_SEC          ((TZ)*3600)
#define DST_SEC         ((DST_MN)*60)
time_t now; //use as 'gmtime(&now);'

//Wifi and MQTT settings stay out of Git. 
#include "SkybadgerStrings.h"
char* defaultHostname        = "espWTR00";
const char* thisID           = "espWTR00";
char* myHostname             = "espWTR00";

WiFiClient espClient;
PubSubClient client(espClient);
volatile bool callbackFlag = 0;

// Create an instance of the server
// specify the port to listen on as an argument
ESP8266WebServer server(80);
ESP8266HTTPUpdateServer updater;

//Create a remote debug object
RemoteDebug Debug;

//Hardware device system functions - reset/restart etc
EspClass device;
ETSTimer timer, timeoutTimer;
volatile bool newDataFlag = false;
volatile bool timeoutFlag = false;
volatile bool timerSet = false;

const int MAXDATA = 6; //6 lots of 10 second data is 1 minute
const int samplesPerMinute = 60/10;

//Function definitions
void onTimer(void); //Core loop timer
void onTimoutTimer(void);//MQTT connection recovery timer
void callback(char* topic, byte* payload, unsigned int length) ;
float calcDewpoint(float t, float h);
uint32_t inline ICACHE_RAM_ATTR myGetCycleCount();
void inline ICACHE_RAM_ATTR windPulseCounter(void);
void inline ICACHE_RAM_ATTR rainPulseCounter(void);
void publishHealth();
void publishSensors();

//Humidity, temperature but not pressure
#include "HTU21D.h"
HTU21D htu;
bool htuPresent = false;
float humidity = 0.0F;
float dewpoint = 0.0F;
float htuTemperature = 0.0f;

//Pressure, tempb, 
//BMP 280 device uses sensor library 
#include "i2c.h"
#include "i2c_BMP280.h"
BMP280 bmp280;
float pressure = 0.0F;
float bmpTemperature = 0.0f;
bool bmpPresent = false;

//Pulse counter for the wind speed
//Interrupt handler variables
const byte windPulseCounterPin = 14;
volatile int windCtr = 0;
volatile boolean windDataFlag = false;

//Acquired data storage and processing 
#define ANEMOMETER_DIAMETER 200

bool windSpeedPresent = false;
const float windSpeedFactor = 1.492; //MPH per click per sec for this set of cups. 
int windData[MAXDATA];
float windAvg = 0;
int windPeak60Rate = 0;

//Todo - update with voltage table mapped to resistances and directions 
bool windDirnPresent = false;
const float ADCRangeCount = 3.30F/1024; //Inbuilt ADC is 10bit, 2mV per digit, only available on ESP8266-12E and upwards
const float precision = ADCRangeCount * 2/100;
float windBearings[2][16] = {{ 0,   22.5,  45,   67.5,  90,  112.5, 147.5, 180,  202.5, 225, 247.5, 270,  292.5, 315,  337.5 }, 
                             { 232, 389,   1000, 753,   820, 112,   140,   30,    43,   16,    24,  8,    66,    58,    280  }};
//char* cardinals [16]   =   { "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W",  "WNW", "NW", "NNW" };
float windDirn = 0.0F;

//#include <Adafruit_ADS1015.h>
//// Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
//Adafruit_ADS1015 ads;     /* Use this for the 12-bit version */
//bool adcPresent = false;

//Rain pulse counter
const float rainTipperSize = 0.01F;
float rainAvg = 0;
int rainPeak60Rate = 0;
int rainData[MAXDATA];
bool rainPresent = false;

//Rain pulse counter Interrupt handler variables
const byte rainPulseCounterPin = 12;
volatile int rainCtr = 0;

#include "Skybadger_common_funcs.h"

//Web Handler function definitions
#include "ESP8266_Weatherhandlers.h"

extern void handleRoot(void);
extern void handleNotFound(void);
extern void handleStatusGet(void);
extern void handleWeatherGet(void);
extern void handleWindDirnGet(void);
extern void handleWindSpeedGet(void);
extern void handleRainGet(void);
extern void handleTemperatureGet(void);
extern void handlePressureGet(void);
extern void handleHumidityGet(void);
#if defined SQM_METER_INCLUDED
extern void handleSkyTempGet(void);
extern void handleSkyBrightnessGet(void);
#endif

//Main processing loop
static byte lastMeasurement = TRIGGER_TEMP_MEASURE_NOHOLD;

void loop()
{
  String timestamp;
  String output;
  static int loopCount = 0;
  static boolean pinState = false;
  static int rainDelayCtr = 0;
  static int windDelayCtr = 0;
  static int rainLoopIndex = 0;
  static int windLoopIndex = 0;      
  
  if( newDataFlag == true ) //every second
  {    
    getTimeAsString2( timestamp ) ;
    debugI( "Time: %s", timestamp.c_str() );
    //Serial.println( getTimeAsString( timestamp ) )
    
    //Only sample humidity < once per 50ms 
    //Doing this makes it every two seconds for each type.
    if( htuPresent )
    {
      if( lastMeasurement == TRIGGER_TEMP_MEASURE_NOHOLD ) 
      {
        int16_t temp = htu.readMeasurement( TRIGGER_TEMP_MEASURE_NOHOLD );
        htuTemperature = htu.rawToActualTemperature( temp );
        
        debugV( "Temp: %f", htuTemperature );
        htu.requestMeasurement( TRIGGER_HUMD_MEASURE_NOHOLD );
      }
      else
      {
        uint16_t hum = htu.readMeasurement( TRIGGER_HUMD_MEASURE_NOHOLD );
        humidity = htu.rawToActualHumidity( hum );      
        debugV( "Humidity: %f", humidity );
        htu.requestMeasurement( TRIGGER_TEMP_MEASURE_NOHOLD );  
      }
      lastMeasurement = (lastMeasurement == TRIGGER_TEMP_MEASURE_NOHOLD)? TRIGGER_HUMD_MEASURE_NOHOLD:TRIGGER_TEMP_MEASURE_NOHOLD;

      //Update dewpoint
      dewpoint = calcDewpoint( htuTemperature, humidity );
      debugV( "Dewpoint: %f", dewpoint );
    }
    
    //Pressure - turn into dew point info
    if( bmpPresent )
    {
      bmp280.awaitMeasurement();
      bmp280.getTemperature( bmpTemperature );
      bmp280.getPressure( pressure );
      debugV("Pressure: %f", pressure );   
      bmp280.triggerMeasurement();//kick of the next one
    }
    else
    {
      pressure = 103265.0;    
    }
      
    //Rain 
    if( rainPresent && rainDelayCtr++ >= 10  ) //every 10 secs
    {
      int loopPeak = 0;
      
      //Only process every 10 seconds or so to allow rain counts to accumulate. 
      
      //Get the data and reset the ctr
      rainData[ rainLoopIndex ] = rainCtr;
      debugV( "Rain (counts): %i, loop %i", rainCtr, rainLoopIndex );
      rainLoopIndex++;
      rainCtr = 0;
      
      if( rainLoopIndex >= MAXDATA )
      {
        //Find the 60-second peak.
        rainAvg = 0.0;
        loopPeak = 0;
        for ( int i=0; i< MAXDATA; i++ )
        {
          if( rainData[ i ] >= loopPeak )
          {
            loopPeak = rainData[ i ];
          }
          debugV( "Rain[%i]: %d", i, rainData[ i ] );
          rainAvg += (float) rainData[ i ];
        }
        rainAvg /= MAXDATA; //Get the rain rate per second. 
        rainPeak60Rate = loopPeak;
        debugD( "Rain (peak) found: %i", loopPeak );        
        rainLoopIndex %= MAXDATA;              
      }
      rainDelayCtr %= 10;
    }
    
    //Wind - capture the interrupt counts every period. 
    if( windSpeedPresent && windDelayCtr++ >= 10 )
    {
      static int windLoopIndex = 0; // Count over 10 second periods - 1 second is too short for slow wind speeds. 
      int loopPeak = 0;

      debugV("Wind speed (counts) : %i, loop %i ", windCtr, windLoopIndex );
      //disable interrupts  
      //capture value
      windData[windLoopIndex] = windCtr;
      windCtr = 0;
      //enable interrupts
      windLoopIndex++;

      if( windLoopIndex >= MAXDATA )       
      {  
          loopPeak = 0;
          windAvg = 0.0;
          for( int i=0; i< MAXDATA; i++ )
          {
            windAvg += (float) windData[ i ];
            if ( windData[ i ] >= loopPeak )
            {
              loopPeak = windData[i];
            } 
            debugV("Wind speed raw[%i]: %d ", i, windData[ i ] );
          }
          windPeak60Rate = loopPeak;                 
          windAvg /= MAXDATA;
        debugD("Wind speed 10 sec counts avg: %f, peak %i ", windAvg, windPeak60Rate );
        windLoopIndex %= MAXDATA;
      }
      windDelayCtr %= 10;
    }
  
    //Wind direction
    //Auto-scaled to 360.0 degrees;
    if( windDirnPresent) 
    {
      int i=0;
      int adcReading = 0;
      int target = 0;
      int precision = 0; 
      int localDirn = -1;
      
      //float adcReading = ads.readADC_SingleEnded( 0 );
      adcReading = analogRead( A0 );
      //debugV("ADC (wind dirn): %u", adcReading );
      for ( i = 0; i< 16; i++)
      { 
        target = (int) windBearings[1][i];
        //We need precision wide enough to allow for voltage variations due to temperature etc while still allowing discrete separation between 
        //direction values. Smallest separation is between 8 and 16.
        precision = max( int( target * 4/100), 4 );
        //debugV( "Comparing %i with %u at precision +/- %i ", target, adcReading, precision ); 
        if( (adcReading >= ( target - precision ))  && ( adcReading <= ( target + precision )) )
        {
          windDirn = windBearings[0][i];          
          debugD("Wind Dirn (bearing) found: %f at ADC: %i", windDirn, adcReading );
          break;
        }
      }
    }  
    //Reset timing flag
    newDataFlag = false;  
  }
  //Webserver handling.
  server.handleClient();
 
  //MQTT housekeeping.
  if ( client.connected() ) 
  {
    if ( callbackFlag )
    {
      publishHealth();
      publishSensors();
      callbackFlag = false;
    }
    client.loop();
  }
  else 
  {
    reconnectNB();
    client.subscribe(inTopic);
  }

  // Remote debug over WiFi
  Debug.handle();
}

/* MQTT callback for subscription and topic.
 * Only respond to valid states ""
 * Publish under ~/skybadger/sensors/<sensor type>/<host>
 * Note that messages have an maximum length limit of 18 bytes - set in the MQTT header file. 
 */
void callback(char* topic, byte* payload, unsigned int length) 
{  
  //set callback flag
  callbackFlag = true;  
}

 void publishHealth( void )
 {
  String outTopic;
  String output;
  String timestamp;
  
  //checkTime();
  getTimeAsString( timestamp );

  //publish to our device topic(s)
  DynamicJsonBuffer jsonBuffer(256);
  JsonObject& root = jsonBuffer.createObject();
  
  root["time"] = timestamp;    
  root["hostname"] = myHostname;
  root["message"] = "weather sensor listening";
  root.printTo( output );

  //Put a notice out regarding device health
  outTopic = outHealthTopic;
  outTopic.concat( myHostname );
  client.publish( outTopic.c_str(), output.c_str() );  
  Serial.printf( "topic: %s, published with value %s \n", outTopic.c_str(), output.c_str() );    
  Serial.print( outTopic );Serial.println( output );
}
 
//MQTT publishing data 
 void publishSensors( void )
 {
  String outTopic;
  String output;
  String timestamp;
  
  //checkTime();
  getTimeAsString( timestamp );

  //publish to our device topic(s)
  DynamicJsonBuffer jsonBuffer(256);
  //JsonObject& root = jsonBuffer.createObject();
  
  if( htuPresent )
  {
    JsonObject& root = jsonBuffer.createObject();
    root["time"] = timestamp;    
    root["sensor"] = "HTU21D";
    root["humidity"] = humidity;
    root.printTo( output );
    
    outTopic = outSenseTopic;
    outTopic.concat("humidity/");
    outTopic.concat(myHostname);
    if ( !client.publish( outTopic.c_str() , output.c_str() ) )
      Serial.print( "failed to publish HTU21D humidity sensor measurement ");
    else
      Serial.println( "HTU21D humidity sensor measurement published");
    Serial.print( outTopic );Serial.println( output );

    root.remove("humidity");
    output = ""; //reset 
    root["sensor"] = "HTU21D";
    root["time"] = timestamp;
    root["temperature"] = htuTemperature;
    root.printTo( output );

    outTopic = outSenseTopic;
    outTopic.concat("temperature/");
    outTopic.concat(myHostname);
    if ( !client.publish( outTopic.c_str(), output.c_str() ) )
      Serial.print( "failed to publish HTU21D temperature sensor measurement ");    
    else
      Serial.println( "HTU21D temperature sensor measurement published");
    Serial.print( outTopic );Serial.println( output ); 

    root.remove("temperature");
    output = ""; //reset 
    
    root["dewpoint"] = dewpoint;
    root.printTo( output );

    outTopic = outSenseTopic;
    outTopic.concat("dewpoint/");
    outTopic.concat(myHostname);
    if ( !client.publish( outTopic.c_str(), output.c_str() ) )
      Serial.print( "failed to publish HTU21D dewpoint sensor measurement ");    
    else
      Serial.println( "HTU21D dewpoint sensor measurement published");
    Serial.print( outTopic );Serial.println( output ); 
    
    output = ""; //reset 
    root.remove("dewpoint");
  }
  
  if( bmpPresent )
  {
    JsonObject& root = jsonBuffer.createObject();
    output="";//reset
    root["sensor"] = "bmp280";
    root["time"] = timestamp;
    root["pressure"] = pressure;
    root.printTo( output );
    
    outTopic = outSenseTopic;
    outTopic.concat("pressure/");
    outTopic.concat(myHostname);

    if ( !client.publish( outTopic.c_str(), output.c_str() ) )        
      Serial.print( "Failed to publish BMP280 pressure sensor measurement ");    
    else    
      Serial.println( "BMP280 pressure sensor measurement published");
    Serial.print( outTopic );Serial.println( output );    

    output = "";
    root.remove("pressure");
    root["temperature"] = bmpTemperature;
    root.printTo( output );

    outTopic = outSenseTopic;
    outTopic.concat("temperature/");
    outTopic.concat(myHostname);
    
    if ( !client.publish( outTopic.c_str(), output.c_str() ) )
    {
      Serial.print( "Failed to publish BMP280 temperature sensor measurement ");    
    }
    else
      Serial.println( "BMP280 temperature sensor measurement published");
    Serial.print( outTopic );Serial.println( output );
  }

  if ( rainPresent) 
  {
    JsonObject& root = jsonBuffer.createObject();
    output="";//reset
    root["sensor"] = "rainrate";
    root["time"] = timestamp;
    root["value"] = 0.01 + rainAvg * rainTipperSize * 3600/10.0; //rainAvg is per 10 seconds, want mm/hr. Use fudge-factor to test. 
    root.printTo( output );

    outTopic = outSenseTopic;
    outTopic.concat("rainrate/");
    outTopic.concat(myHostname);

    if ( !client.publish( outTopic.c_str(), output.c_str() ) )        
      Serial.print( "Failed to publish Rain tipper sensor measurement ");    
    else    
      Serial.println( "Rain measurement published");
    Serial.print( outTopic );Serial.println( output );    

    output = "";
    root["sensor"] = "rainpeakrate";
    root["value"] = rainPeak60Rate * rainTipperSize * 3600/10.0 ;//rainPeak is captured for the 10 second periods in each minute. 
    root.printTo( output );

    if ( !client.publish( outTopic.c_str(), output.c_str() ) )        
      Serial.print( "Failed to publish Rain tipper peak measurement ");    
    else    
      Serial.println( "Rain peak measurement published");
    Serial.print( outTopic );Serial.println( output );    
  }
  
  if (windSpeedPresent) 
  {
    JsonObject& root = jsonBuffer.createObject();
    output="";//reset
    root["sensor"] = "windspeed";
    root["time"] = timestamp;
    root["value"] = windAvg/10.0 * windSpeedFactor; //Avg is avg in a minute across 6 10-second buckets. 
    root.printTo( output );
        
    outTopic = outSenseTopic;
    outTopic.concat("windspeed/");
    outTopic.concat(myHostname);

    if ( !client.publish( outTopic.c_str(), output.c_str() ) )        
      Serial.print( "Failed to publish anemometer sensor measurement ");    
    else    
      Serial.println( "Anemometer measurement published");
    Serial.print( outTopic );Serial.println( output );    

    output = "";
    root["sensor"] = "windgust";
    root["value"] = windPeak60Rate/10.0 * windSpeedFactor;//Peak is peak 10-second count in a minute window 
    root.printTo( output );
    
    outTopic = outSenseTopic;
    outTopic.concat("windgust/");
    outTopic.concat(myHostname);
    
    if ( !client.publish( outTopic.c_str(), output.c_str() ) )        
      Serial.print( "Failed to publish anemometer peak sensor measurement ");    
    else    
      Serial.println( "Anemometer peak measurement published");
    Serial.print( outTopic );Serial.println( output );    
  }
  
  if (windDirnPresent) 
  {
    JsonObject& root = jsonBuffer.createObject();
    output="";//reset
    root["sensor"] = "garagevane";
    root["time"] = timestamp;
    root["value"] = windDirn;
    root["hostname"] = myHostname;

    outTopic = outSenseTopic;
    outTopic.concat("winddirn/");
    outTopic.concat(myHostname);

    root.printTo( output );
    if ( !client.publish( outTopic.c_str(), output.c_str() ) )        
      Serial.print( "Failed to publish Wind dirn sensor measurement ");    
    else    
      Serial.println( "Wind dirn sensor measurement published");
    Serial.print( outTopic );Serial.println( output );    

    output = "";
    root.remove("Direction");  
  }

  //Post a device health message
  output = "Measurement published";
  outTopic = outHealthTopic;
  outTopic.concat( myHostname );
  client.publish( outTopic.c_str(), output.c_str() );  
  Serial.print( outTopic );Serial.print( " published: " ); Serial.println( output );
 }
 
// calculates dew point
// input:   humidity [%RH], temperature in C
// output:  dew point in C
float calcDewpoint(float t, float h)
{
  float temp = 0.0F;
  float logEx;
  logEx = 0.66077 + 7.5 * t / (237.3 + t) + (log10(h) - 2);
  temp = (logEx - 0.66077) * 237.3 / (0.66077 + 7.5 - logEx);
  return temp;
} 

uint32_t inline ICACHE_RAM_ATTR myGetCycleCount()
{
    uint32_t ccount;
    __asm__ __volatile__("esync; rsr %0,ccount":"=a" (ccount));
    return ccount;
}

//Pulse detector for the rain measurement
void inline ICACHE_RAM_ATTR rainPulseCounter(void)
{
    //Count pulses.
    rainCtr++;
}

/*
  Interrupt handler function captures clock counts at rising and falling edges on input pin.
  Sets flag for main loop to calculate pule width. 
  Check for potential issue of loop() taking multiple pulse count lengths. 
  Address using countHandled flag ?
  https://github.com/esp8266/esp8266-wiki/wiki/gpio-registers
  Measured on esP8266-12 to take 300K 160Mhz clocks per loop iteration  - 20ms. 
*/
void inline ICACHE_RAM_ATTR windPulseCounter(void)
{
    //count pulses.
    windCtr++;
}

void setup_wifi()
{
  int zz = 0;
  WiFi.mode(WIFI_STA);
  WiFi.hostname( myHostname );
    
  WiFi.begin(ssid2, password2);
  Serial.print("Searching for WiFi..");
  while (WiFi.status() != WL_CONNECTED) 
  {
      delay(500);
      Serial.print(".");
      if( zz++ > 400 )
        device.restart();
  }
  
  Serial.println("WiFi connected");
  Serial.printf("SSID: %s, Signal strength %i dBm \n\r", WiFi.SSID().c_str(), WiFi.RSSI() );
  Serial.printf("Hostname: %s\n\r",       WiFi.hostname().c_str() );
  Serial.printf("IP address: %s\n\r",     WiFi.localIP().toString().c_str() );
  Serial.printf("DNS address 0: %s\n\r",  WiFi.dnsIP(0).toString().c_str() );
  Serial.printf("DNS address 1: %s\n\r",  WiFi.dnsIP(1).toString().c_str() );

  //Setup sleep parameters
  //wifi_set_sleep_type(LIGHT_SLEEP_T);
  wifi_set_sleep_type(NONE_SLEEP_T);
  delay(5000);
}

void setup()
{
  Serial.begin( 115200, SERIAL_8N1, SERIAL_TX_ONLY);
  Serial.println(F("ESP starting."));
  //gdbstub_init();
  delay(2000); 

  // Connect to wifi 
  setup_wifi();                     

  //Start time
  configTime(TZ_SEC, DST_SEC, timeServer1, timeServer2, timeServer3 );
  Serial.println( "Time Services setup");     

  //Read internal state, apply defaults if we can't find user-set values in Eeprom.
  EEPROM.begin(512);
  //setDefaults();
  //readFromEeprom();    
  delay(2000);    

  //Debugging over telnet setup
  // Initialize the server (telnet or web socket) of RemoteDebug
  //Debug.begin(HOST_NAME, startingDebugLevel );
  Debug.begin( WiFi.hostname().c_str(), Debug.ERROR ); 
  Debug.setSerialEnabled(true);//until set false 
  // Options
  // Debug.setResetCmdEnabled(true); // Enable the reset command
  // Debug.showProfiler(true); // To show profiler - time between messages of Debug
  //In practice still need to use serial commands until debugger is up and running.. 
  debugE("Remote debugger enabled and operating");
  
  //Pulse counter interrupts
  //Rain
  pinMode( rainPulseCounterPin, INPUT_PULLUP );      // set pin to input  
  digitalWrite( rainPulseCounterPin, LOW );         //Turn off internal pullups which would prevent the pulse being detected 
  attachInterrupt( digitalPinToInterrupt( rainPulseCounterPin ), rainPulseCounter, RISING );  
  //attachInterrupt( rainPulseCounterPin, rainPulseCounter, RISING );  
  
  //Wind
  pinMode( windPulseCounterPin, INPUT_PULLUP );           // set pin to input
  digitalWrite( windPulseCounterPin, LOW );        //Turn off internal pullups which would prevent the pulse being detected 
  attachInterrupt( digitalPinToInterrupt( windPulseCounterPin), windPulseCounter, RISING );  
  //attachInterrupt( windPulseCounterPin, windPulseCounter, RISING );  
  
  //Pins mode and direction setup for i2c on ESP8266-12
  //I2C setup SDA pin 0, SCL pin 2 on ESP-01
  //I2C setup SDA pin 5, SCL pin 4 on ESP-12
  //Wire.begin(SDL, SCL);
  Wire.begin( 5, 4);
  Wire.setClock(100000 );//100KHz target rate
  
  Serial.println(F("Pins setup & interrupts attached."));
  
  //Open a connection to MQTT
  DEBUGS1("Starting to configure MQTT connection to :");DEBUGSL1( mqtt_server );
  client.setServer( mqtt_server, 1883 );
  Serial.printf(" MQTT settings id: %s user: %s pwd: %s\n", thisID, pubsubUserID, pubsubUserPwd );
  client.connect( thisID, pubsubUserID, pubsubUserPwd ); 
  //Create a timer-based callback that causes this device to read the local i2C bus devices for data to publish.
  client.setCallback( callback );
  client.subscribe( inTopic );
  publishHealth();
  client.loop();
  DEBUGSL1("Configured MQTT connection");

  //Setup the sensors BMP280, HTU21D, Cup anemometer, microswitch wind direction sensor and relay output rain gauge ( tipper equiv) 
  Serial.print("Probe BMP280: ");
  bmpPresent = bmp280.initialize();
  if ( !bmpPresent ) 
  {
    Serial.println("BMP280 Sensor not found");
  }
  else
  {
    Serial.println("BMP280 Sensor found");
    // onetime-measure:
    bmp280.setEnabled(0);
    bmp280.triggerMeasurement();
  }
  
//HTU
  Serial.print("Probe HTU21D: ");
  htu.begin( );
  htu.setResolution(USER_REGISTER_RESOLUTION_RH11_TEMP11 ); //7ms delay
  
  if( !(htuPresent = htu.requestMeasurement( TRIGGER_TEMP_MEASURE_NOHOLD )) ) 
  {
    Serial.println("HTU21D Sensor not found");
  }
  else
  {
    Serial.println("HTU21D Sensor found");
    delay(50);
    
    // one-time measure: 
    htuTemperature = (float) htu.readMeasurement( TRIGGER_TEMP_MEASURE_NOHOLD );
  }
 
//Setup data arrays for pulse counting sensors.
  for( int i=0; i< MAXDATA; i++)
  {
    windData[i] = 0;
    rainData[i] = 0;
  }

  //setup wind direction
  windDirn = 0;
  windCtr = 0;
  windDirnPresent = true;
  windSpeedPresent = true;
  
  //Setup rain gauge counters. 
  rainCtr = 0;
  rainPresent = true;
  
  //to start with we'll use the built in ADC on A0 so don't need this . 
  //{
  // Setup ADC for wind direction
  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
  // We provide 5v to the wind gauge. Using a 10K resistor to ground and measuring at the top we get a max value of ..
  //ads.begin();
  //ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  //if( ads.readADC_SingleEnded( 0 ) > 0 ) 
  //  adcPresent = true;
  //}
  
  //Setup webserver handler functions
  server.on("/", handleRoot );
  server.onNotFound( handleNotFound ); 
  //TODO add handler code in separate file. 
  if ( windDirnPresent ) 
  {
    server.on("/wind/direction",  HTTP_GET, handleWindDirnGet );
  }
  if( windSpeedPresent )
  {  
    server.on("/wind/speed",      HTTP_GET, handleWindSpeedGet );
    server.on("/wind/peak",      HTTP_GET, handleWindPeakGet );
  }
  if ( rainPresent ) 
  {
    server.on("/rain/rate",            HTTP_GET, handleRainGet );
    server.on("/rain/peak",            HTTP_GET, handleRainPeakGet );
  }
  if ( htuPresent ) 
  {
    server.on("/temperature",     HTTP_GET, handleTemperatureGet );
    server.on("/humidity",        HTTP_GET, handleHumidityGet );
  }
  if( bmpPresent ) 
  {
    server.on("/pressure",        HTTP_GET, handlePressureGet );
    server.on("/humidity",        HTTP_GET, handleHumidityGet );
  }

  //server.on("/SkyTemp",         HTTP_GET, handleSkyTempGet );
  //server.on("/SkyBrightness",   HTTP_GET, handleSkyBrightnessGet );
  updater.setup( &server );
  server.begin();
    
  //Setup timers for measurement loop
  //setup interrupt-based 'soft' alarm handler for periodic acquisition/recording of new measurements.
  ets_timer_setfn( &timer, onTimer, NULL ); 
  ets_timer_setfn( &timeoutTimer, onTimeoutTimer, NULL ); //MQTT non-blocking re-connect timer
  
  //fire timer every 1000 msec
  //Set the timer function first  
  ets_timer_arm_new( &timer, 1000, 1/*repeat*/, 1);
  //ets_timer_arm_new( &timeoutTimer, 2500, 0/*one-shot*/, 1);
 
  //Show welcome message
  Debug.setSerialEnabled(false);

  Serial.println( "Setup complete" ); 
}

//Timer handler for 'soft' interrupt timer. 
void onTimer( void * pArg )
{
  newDataFlag = true;
}

//Used to complete timeout actions. 
void onTimeoutTimer( void* pArg )
{
  //Read command list and apply. 
  timeoutFlag = true;
}
