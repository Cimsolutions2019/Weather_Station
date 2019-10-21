#include <NTPClient.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>

/*
    This sketch establishes a TCP connection to a time client and the MCC Solar Voyager database.
    It sends the sensor values
*/

#include <ESP8266WiFi.h>

#ifndef STASSID
//#define STASSID "CIM_Arduino"
//#define STAPSK  "d26bedfab3"
#define STASSID "CIMSOLUTIONS Guest"
#define STAPSK  "Airport02"
#endif

#define NTP_OFFSET   60 * 60      // In seconds
#define NTP_INTERVAL 60 * 1000    // In miliseconds
#define NTP_ADDRESS  "europe.pool.ntp.org"
 
const char* ssid     = STASSID;
const char* password = STAPSK;

const char* host = "172.16.0.45";

//const char* getHost = "10.5.0.205"; // The data receiver host name (not URL)
//const int httpGetPort = 80; // The data receiver host port
String getReceiverURL = "/"; // The data receiver script

//WiFiServer server(80); 

String data, latestTime;
bool send = false;
float temp,hum,lon,lat;
String gpsTime;
bool active = false;
int research, id = 0;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, NTP_ADDRESS, NTP_OFFSET, NTP_INTERVAL);

void setup()
{
  // initialize serial port:
  Serial.begin(9600);
  while(!Serial);

  // connect to Wi-Fi
  Serial.printf("Connecting to %s ", ssid);
  WiFi.begin(ssid, password);
  // start Time client
  timeClient.begin();
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" connected");
  Serial.println(WiFi.localIP());

  Serial.printf("Web server started, open %s in a web browser\n", WiFi.localIP().toString().c_str());
}


void loop()
{
    // Test message for POST command on serial: 22.6:55.6:4.603:52.506
    // Structure for activating weather station : "status:research:id"
    // Status can be 0 or 1, off and on.
  // check if there is a message on the serial
  while (Serial.available() > 0)
    {
        // convert message to sensor values
        gpsTime = Serial.readStringUntil('Z') + "Z";
        Serial.readStringUntil(':');
        temp = Serial.readStringUntil(':').toFloat();
        hum = Serial.readStringUntil(':').toFloat();
        lon = Serial.readStringUntil(':').toFloat();
        lat = Serial.readStringUntil('\n').toFloat();
        send = true;
    }
  if(send){
    postData();
    send = false;
  }
}

void postData(){
    // start wifi client for sending
    WiFiClient client;
    // get the current time
    while(!timeClient.update()) {
      timeClient.forceUpdate();
    }
    // compensate for offset on the world
    timeClient.setTimeOffset(7200);
    // create json structure for data
    DynamicJsonDocument doc(1024);
    JsonObject humi  = doc.createNestedObject("humidity");
    humi["value"] = hum;
    doc["latitude"] = lat;
    doc["longitude"] = lon;
    JsonObject tempi = doc.createNestedObject("temperature");
    tempi["value"] = temp;
    latestTime = timeClient.getFormattedDate();
    //doc["time"] = latestTime;
    doc["time"] = gpsTime;
    serializeJson(doc, Serial);
    Serial.println();
  
    Serial.printf("\n[Connecting to %s ... ", host);
    if (client.connect(host, 8080))
    {
      Serial.println("connected]");
      delay(250);
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
      delay(100);
      while (client.connected() || client.available())
      {
        if (client.available())
        {
          String line = client.readStringUntil('\n');
          Serial.println(line);
          delay(100);
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
}
