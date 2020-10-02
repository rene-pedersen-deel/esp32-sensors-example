//Libraries
#include <DHT.h>
#include <WiFi.h>
#include <PubSubClient.h>

#include "config.h"

char payload[100];
char topic[150];
// Space to store values to send
char str_sensor[10];

#define DHTPIN 15         // what pin we're connected to
#define DHTTYPE DHT22     // DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE); //// Initialize DHT sensor for normal 16mhz Arduino

#define pirPin 14
#define motionLED 22


WiFiClient wifiClient;
PubSubClient client(wifiClient);

void setup() {
    Serial.begin(115200);
    dht.begin();

    pinMode(motionLED, OUTPUT);
    pinMode(pirPin, INPUT);

    WiFi.begin(WIFISSID, WIFIPASSWORD);

    Serial.println();
    Serial.print("Wait for WiFi...");

    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        delay(500);
    }

    Serial.println("");
    Serial.println("WiFi Connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    client.setServer(mqttBroker, 1883);
    client.setCallback(callback);
}


void loop() {
    if (!client.connected()) {
        reconnect();
    }
    client.loop();

    readLightLevel();
    measureTempHum();
    detectMotion();
}


class myClass {
    private:
        float currentVal = 1;
        float newVal = 1;
        unsigned long lastRead = millis();
};


const int lightSensorPin = 34;
float currentLightLevel = 1;
unsigned long lastReadLightLevel = 0;
void readLightLevel() {
    if ( millis() < lastReadLightLevel ) lastReadLightLevel = 0;
    if ( millis() - lastReadLightLevel < 2000 ) return;

    float newLightLevel = analogRead(lightSensorPin); // read the current light levels

    if ( currentLightLevel == 0 ) {
        currentLightLevel = newLightLevel;
        return;
    }
    float change = abs( newLightLevel - currentLightLevel ) / currentLightLevel;
    // Serial.print("change: ");
    // Serial.println(change);
    if ( change >= 0.05  || millis() - lastReadLightLevel > 15*60*1000  ) {
        lastReadLightLevel = millis();
        currentLightLevel = newLightLevel;
        Serial.print("LightLevel: ");
        Serial.println(newLightLevel);
        char lightString[8];
        dtostrf(newLightLevel, 1, 2, lightString);
        client.publish("sensor/light/hallway", lightString);
    }
}



unsigned long lastReadTempHum = 0;
float currentTemperature = 1;
float currentHumidity = 1;
void measureTempHum() {
    if ( millis() < lastReadTempHum ) lastReadTempHum = 0; // Reset when overflow
    if ( millis() - lastReadTempHum < 2000 ) return;

    //Read data and store it to variables hum and temp
    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();

    if ( currentTemperature == 0 ) {
        currentTemperature = temperature;
        return;
    }
    if ( currentHumidity == 0 ) {
        currentHumidity = humidity;
        return;
    }

    float temperatureChange = abs( temperature - currentTemperature ) / currentTemperature;
    if ( temperatureChange >= 0.01 || millis() - lastReadTempHum > 1*60*1000 ) {
        currentTemperature = temperature;
        lastReadTempHum = millis();
        Serial.print("Temp: ");
        Serial.print(temperature);
        Serial.println(" Celsius");

        char tempString[8];
        dtostrf(temperature, 1, 2, tempString);
        client.publish("sensor/temp/hallway", tempString);
    }

    float humidityChange = abs( humidity - currentHumidity ) / currentHumidity;
    // Serial.print("Humidity change: ");
    // Serial.println( humidityChange );
    if ( humidityChange >= 0.01 ) {
        currentHumidity = humidity;
        Serial.print("Humidity: ");
        Serial.println(humidity);

        char humString[8];
        dtostrf(humidity, 1, 2, humString);
        client.publish("sensor/hum/hallway", humString);
    }
}



bool newMotionState = false;
bool currentMotionState = false; // We start with no motion detected.
void detectMotion() {
    // Read out the pirPin and store as newMotionState:
    int readMotionState = digitalRead(pirPin);
    if ( readMotionState == 0 ) {
        newMotionState = false;
    } else {
        newMotionState = true;
    } 
    if ( newMotionState == currentMotionState ) {
        return;
    }
    // If motion is detected (pirPin = HIGH), do the following:
    if (newMotionState == true) {
        digitalWrite(motionLED, HIGH); // Turn on the on-board LED.
        // Change the motion state to true (motion detected):
        Serial.println("Motion detected!");
        currentMotionState = true;
        client.publish("sensor/motion/hallway", "on");
    }
    // If no motion is detected (pirPin = LOW), do the following:
    else {
        digitalWrite(motionLED, LOW); // Turn off the on-board LED.
        // Change the motion state to false (no motion):
        Serial.println("Motion ended!");
        currentMotionState = false;
        client.publish("sensor/motion/hallway", "off");
    }
}

// MQTT

void callback(char *topic, byte *payload, unsigned int length) {
    char p[length + 1];
    memcpy(p, payload, length);
    p[length] = NULL;
    String message(p);
    Serial.write(payload, length);
    Serial.println(topic);
}



void reconnect() {
    // Loop until we're reconnected
    while (!client.connected())
    {
        Serial.println("Attempting MQTT connection...");

        // Attemp to connect

        if (client.connect("esp32-02", "kong", "kong"))
        {
            Serial.println("Connected-------------------------------------------");
        }
        else
        {
            Serial.print("Failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 2 seconds");
            // Wait 2 seconds before retrying
            delay(2000);
        }
    }
}
