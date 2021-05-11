#if ! defined _ESP8266_Weatherhandlers_h_
#define _ESP8266_Weatherhandlers_h_
extern ESP8266WebServer server;

  /*
   * Web server handler functions
   */
  void handleNotFound(void)
  {
  String message = "URL not understood\n";
  message.concat( "Simple status read: http://");
  message.concat( myHostname );
  message.concat ( "\n");
  message.concat( "Temperature read: http://");
  message.concat( myHostname );
  message.concat ( "/temperature \n");
  message.concat( "Pressure read: http://");
  message.concat( myHostname );
  message.concat ( "/pressure \n");
  message.concat( "Wind speed read: http://");
  message.concat( myHostname );
  message.concat ( "/wind/speed \n");
  message.concat( "Wind peak60 speed read: http://");
  message.concat( myHostname );
  message.concat ( "/wind/gust \n");
  message.concat( "Wind bearing read: http://");
  message.concat( myHostname );
  message.concat ( "/wind/direction \n");
  message.concat( "Rain gauge read: http://");
  message.concat( myHostname );
  message.concat ( "/rain \n");
  message.concat( "Rain peak60 read: http://");
  message.concat( myHostname );
  message.concat ( "/rain/peak \n");
  server.send(404, "text/plain", message);
  }
  
  //Return sensor status
  void handleRoot(void )
  {
    String timeString = "", message = "";
    DynamicJsonBuffer jsonBuffer(256);
    JsonObject& root = jsonBuffer.createObject();

    root["time"] = getTimeAsString( timeString );
    if( htuPresent )
    {
      root["Humidity"] = humidity;
      root["Dewpoint"] = dewpoint;
    }

    if( bmpPresent )  
    {
      root["pressure"] = pressure;
    }
    
    //Add in wind, rain and rain bearing
    root["wind bearing"]              = windDirn; //index 0-15
    root["Wind speed"]                = (double) ( windAvg * windSpeedFactor );
    root["wind speed 60 second peak"] = windPeak60Rate * windSpeedFactor;
    root["rain rate"]                 = (float) (rainAvg/10.0 * rainTipperSize * 3600); //mm/hr
    root["rain peak"]                 = (float) (rainPeak60Rate/10.0 * rainTipperSize * 3600); //mm/hr
        
    JsonArray& temps = root.createNestedArray("temperatures");
    if ( bmpPresent )  
    {
      temps.add( bmpTemperature );            
    }
    if( htuPresent ) 
    {
      temps.add( htuTemperature );        
    }
    //root.printTo( Serial );
    root.printTo(message);
    server.send(200, "application/json", message);
  }
  
  
  void handleWindDirnGet( void )
  {
    String timeString = "", message = "";
    DynamicJsonBuffer jsonBuffer(256);
    JsonObject& root = jsonBuffer.createObject();

    root["time"] = getTimeAsString( timeString );
    if( windDirnPresent )
      root["Direction"] = (float) windDirn;
    else
      root["Message"] = "Wind vane not present";

    root.printTo( Serial );
    root.printTo(message);
    server.send(200, "application/json", message);  
  }
  
  void handleWindSpeedGet( void )
  {
    String timeString = "", message = "";
    DynamicJsonBuffer jsonBuffer(256);
    JsonObject& root = jsonBuffer.createObject();

    root["time"] = getTimeAsString( timeString );
    if( windSpeedPresent )
      root["Speed"] = (float)(windAvg/10.0 * windSpeedFactor);
    else
      root["Message"] = "Anemometer not present";

    root.printTo( message);
    debugD("%s", message.c_str() );
    server.send(200, "application/json", message);  
  }

  void handleWindPeakGet( void )
  {
    String timeString = "", message = "";
    DynamicJsonBuffer jsonBuffer(256);
    JsonObject& root = jsonBuffer.createObject();

    root["time"] = getTimeAsString( timeString );
    if( windSpeedPresent )
      root["Peak60"] = (float)(windPeak60Rate/10.0 * windSpeedFactor);
    else
      root["Message"] = "Anemometer not present";

    root.printTo(message);
    debugD( "%s", message.c_str() );
    server.send(200, "application/json", message);  
  }

  void handleRainGet( void )
  {
    String timeString = "", message = "";
    DynamicJsonBuffer jsonBuffer(256);
    JsonObject& root = jsonBuffer.createObject();

    root["time"] = getTimeAsString( timeString );
    if( rainPresent )
      root["rain rate"] = (float) rainAvg * rainTipperSize/10.0 * 3600;
    else
      root["Message"] = "Rain gauge not present";

    root.printTo(message);
    debugD( "%s", message.c_str() );
    server.send(200, "application/json", message);  
  }

  void handleRainPeakGet( void )
  {
    String timeString = "", message = "";
    DynamicJsonBuffer jsonBuffer(256);
    JsonObject& root = jsonBuffer.createObject();

    root["time"] = getTimeAsString( timeString );
    if( rainPresent )
      root["Peak60"] = (float) rainPeak60Rate/10.0 * rainTipperSize * 3600;
    else
      root["Message"] = "Rain gauge not present";

    root.printTo(message);
    debugD( "%s", message.c_str() );
    server.send(200, "application/json", message);  
  }
  
  void handleTemperatureGet( void )
  {
    String timeString = "", message = "";
    DynamicJsonBuffer jsonBuffer(256);
    JsonObject& root = jsonBuffer.createObject();

    root["time"] = getTimeAsString( timeString );
    if( bmpPresent )
      root["Temperature"] = (float) bmpTemperature;
    else
      root["Message"] = "BMP sensor not present";

    root.printTo(message);
    debugD( "%s", message.c_str() );
    server.send(200, "application/json", message);      
  }
  
  void handlePressureGet( void )
  {
    String timeString = "", message = "";
    DynamicJsonBuffer jsonBuffer(256);
    JsonObject& root = jsonBuffer.createObject();

    root["time"] = getTimeAsString( timeString );
    if( bmpPresent )
      root["Pressure"] = (float) pressure;
    else
      root["Message"] = "BMP sensor not present";

    root.printTo( Serial );
    root.printTo(message);
    server.send(200, "application/json", message);      
  }

  void handleHumidityGet( void )
  {
    String timeString = "", message = "";
    DynamicJsonBuffer jsonBuffer(256);
    JsonObject& root = jsonBuffer.createObject();

    root["time"] = getTimeAsString( timeString );
    if( htuPresent )
    {
      root["Humidity"] = (float) humidity;
      root["Dewpoint"] = (float) dewpoint;
    }
    else
      root["Message"] = "HTU21D sensor not present";

    root.printTo( Serial );
    root.printTo(message);
    server.send(200, "application/json", message);      
  }
  
#if defined SQM_METER_INCLUDED  
  void handleSkyTempGet( void )
  {
    String timeString = "", message = "";
    DynamicJsonBuffer jsonBuffer(256);
    JsonObject& root = jsonBuffer.createObject();

    root["time"] = getTimeAsString( timeString );
    if( mlxPresent )
    {
      root["Sky Temperature"] = (float) skyTemperature;
      root["Ambient Temperature"] = (float) ambientTemperature;
    }
    else
      root["Message"] = "MLX90614 sensor not present";

    root.printTo( Serial );
    root.printTo(message);
    server.send(200, "application/json", message);      
  }

 void handleSkyBrightnessGet( void )
  {
    String timeString = "", message = "";
    DynamicJsonBuffer jsonBuffer(256);
    JsonObject& root = jsonBuffer.createObject();

    root["time"] = getTimeAsString( timeString );
    if( tslPresent )
    {
      root["Brightness"] = (float) lux;
    }
    else
      root["Message"] = "TLS2591 sensor not present";

    root.printTo( Serial );
    root.printTo(message);
    server.send(200, "application/json", message);      
  }
 #endif 
 
 void handleStatusGet( void)
 {
    String timeString = "", message = "";
    DynamicJsonBuffer jsonBuffer(512);
    JsonObject& root = jsonBuffer.createObject();
    //Update the errorr message handling.
    // JsonArray errors = root.createArray( "errors" );
    
    root["time"] = getTimeAsString( timeString );
    
    if( windDirnPresent )
    {
      root["Direction"] = (float) windDirn;
    }
    else
    {
      root["Error Message"] = "Wind vane not present";
      //errors.add( "Wind vane not present" );
    }
      
    if( windSpeedPresent )
    {
      root["Wind"] = (float)(windAvg/10.0 * windSpeedFactor);
      root["Wind gust"] = (float) windPeak60Rate/10.0 * windSpeedFactor;
    }
    else
      root["Message"] = "Anemometer not present";

    if( rainPresent )
    {
      root["Rain"] = (float) (rainAvg * rainTipperSize/10.0 * 3600);
      root["Rain Peak"] = (float) (rainPeak60Rate * rainTipperSize/10.0 * 3600);
    }
      
    else
      root["Message"] = "Rain gauge not present";

    if( bmpPresent )
    {
      root["Temperature"] = (float) bmpTemperature;
      root["Pressure"] = (float) pressure;
    }
    else
      root["Message"] = "BMP sensor not present";
   
    if( htuPresent )
    {
      root["Humidity"] = (float) humidity;
      root["Dewpoint"] = (float) dewpoint;
    }
    else
      root["Message"] = "HTU21D sensor not present";

    root.printTo( message );
    debugD( "%s", message.c_str() );
    server.send(200, "application/json", message);  
 }
#endif
