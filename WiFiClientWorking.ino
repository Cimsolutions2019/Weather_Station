#include <NTPClient.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>

/*
    This sketch establishes a TCP connection to a time client and the MCC Solar Voyager database.
    It sends the sensor values
*/

#include <ESP8266WiFi.h>

#ifndef STASSID
#define STASSID "CIM_Arduino"
#define STAPSK  "d26bedfab3"
#endif

#define NTP_OFFSET   60 * 60      // In seconds
#define NTP_INTERVAL 60 * 1000    // In miliseconds
#define NTP_ADDRESS  "europe.pool.ntp.org"
 
const char* ssid     = STASSID;
const char* password = STAPSK;

const char* host = "172.16.0.45";

String data;
boolean send = false;
float temp,hum,lon,lat;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, NTP_ADDRESS, NTP_OFFSET, NTP_INTERVAL);

void setup()
{
  Serial.begin(115200);
  Serial.println();

  Serial.printf("Connecting to %s ", ssid);
  WiFi.begin(ssid, password);
  timeClient.begin();
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" connected");
}


void loop()
{
    // 22.6:55.6:4.603:52.506
  while (Serial.available() > 0)
    {
        temp = Serial.readStringUntil(':').toFloat();
        hum = Serial.readStringUntil(':').toFloat();
        lon = Serial.readStringUntil(':').toFloat();
        lat = Serial.readStringUntil('\n').toFloat();
        send = true;
    }
  if(send){
      while(!timeClient.update()) {
        timeClient.forceUpdate();
      }
      timeClient.setTimeOffset(7200);
      DynamicJsonDocument doc(1024);
      JsonObject humi  = doc.createNestedObject("humidity");
      humi["value"] = hum;
      doc["latitude"] = lat;
      doc["longitude"] = lon;
      JsonObject tempi = doc.createNestedObject("temperature");
      tempi["value"] = temp;
      doc["time"] = timeClient.getFormattedDate();
      serializeJson(doc, Serial);
      Serial.println();

    WiFiClient client;
  
    Serial.printf("\n[Connecting to %s ... ", host);
    if (client.connect(host, 8080))
    {
      Serial.println("connected]");
  
      Serial.println("[Posting sensor values]");
      client.println("POST /research/3/voyager/1 HTTP/1.1");
      client.println("content-type: application/json");
      client.print("content-length: ");
      client.println(measureJson(doc));
      client.print("Host: ");
      client.println(host);
      client.println("Connection: close");
      client.println();
      serializeJson(doc, client);
      client.println();
      //client.println(data);
      Serial.println("[Response:]");
      while (client.connected() || client.available())
      {
        if (client.available())
        {
          String line = client.readStringUntil('\n');
          Serial.println(line);
        }
      }
      client.stop();
      Serial.println("\n[Disconnected]");
    }
    else
    {
      Serial.println("connection failed!]");
      client.stop();
    }
    send = false;
  }
}
