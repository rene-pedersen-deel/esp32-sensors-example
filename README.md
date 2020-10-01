# esp32-sensors-example
Use an ESP32 to interface a DHT22, a photo resistor and a PIR sensor.  

# config.h
This file must contain the following definitions (replace all the xxxx with your definitions):  
  
#define WIFISSID "xxxx"                        // Put your WifiSSID here  
#define WIFIPASSWORD "xxxx"                    // Put your wifi password here  
 
#define MQTT_CLIENT_NAME "xxxx"                // MQTT client Name, please enter your own 8-12   alphanumeric character ASCII string;  
#define MQTT_CLIENT_PASSWORD "xxxx"            // MQTT client Name, please enter your own 8-12   alphanumeric character ASCII string;
char mqttBroker[] = "192.168.1.70";            // Address of the MQTT broker  
