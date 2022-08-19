<h3>ESP8266_Weather.ino </h3>
<p> ASCOM ALPACA WEather station over WiFi</p
<p>This application runs on the ESP8266-12 wifi-enabled SoC device to implement a weather station to replace the Davis or Cumulus type of weather station from Maplin and other sources that eventually fails due to water ingress, battery leakage, bearing seizure and the like <br/>
This app is meant to replace the brains of th systems with an ESP and re-use the sensor components where possible - like the cup anemometer and direction sensor, but replaces the temp, pressure and humidity sensors with commercial i2C devices like the BMP280 and HTDU21. I fitted all this into the same plastic envelope of the original weather station to re-use the sheltered environment for temperature sensing. </p>

<p>The unit is setup for i2c IO operation ( pressure, temperature, humidity ) and is connected to 
<ul>
<li>SCL on GPIO4, </li>
<li>SDA on GPIO5,  </li>
<li>Tx on GPIO2</li>
<li>Rx on GPIO 3, </li> </ul>

reads the direction sensor via the built-in ADC on GPIO12 and measures the Hygreon sensor rain-detector pulse counter on pin GPIO13. <br/>
The device only supports use  of ESP8266-12 due to needing the built-in ADC which, while present, isn't brought out on the -01 device <br/>
In my arrangement, Node-red flows are used to listen for and graph the updated readings in the dashboard UI. SO this device listens for heartbeat topics every 60 seconds and responds to the device-specific heartbeat topic.<br/>
The firmware image can be updated remotely by the HttpUpdater process. </p>
  
<h3>Dependencies</h3>
<ul>
  <li>Arduino 1.8+ </li>
  <li>ESP8266 V2.4+</li>
<li>Arduino MQTT client (https://pubsubclient.knolleary.net/api.html) - used for report state to the Node-red device health monitor</li>
<li>Arduino JSON library (pre v6)  - used to parse and respond to REST queries. </li>
<li>ASCOM_COMMON library - see other repo here. Used to create standard response templates and handle ALPACA UDP queries. </li>
<li>Remote Debug library  - Used to access via telnet for remote debugging</li>
</ul>


<h3>Testing</h3>
<p>Access by serial port  - Tx only is available from device at 115,600 baud at 3.3v. THis provides debug output .<br/>
Wifi is used for REST-ful web access <br/>
ESP8266HttpServer is used to service web requests on port 80 <br/>
Use http://ESPwtr01/setup to enter setup page for the device. <br/>
</p>

<h3>Use</h3>
<p> <br/>
</P>


<h3>ToDo </h3>
<p>This has now been running in the field for the last three months. 
</p>
