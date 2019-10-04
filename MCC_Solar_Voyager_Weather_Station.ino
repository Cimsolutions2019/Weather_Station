#include <DHT.h>
/*
  Arduino weerstation voor MCC Solar Voyager
  Verstuurt luchtvochtigheid en temperatuur naar een WiFi-module die het doorstuurt naar de database.
*/

#define DHTPIN 2     // what pin we're connected to
#define DHTTYPE DHT22   // type sensor

DHT dht(DHTPIN, DHTTYPE);

long previousMillis = 0;        // will store last time voyager was updated
 
// the follow variables is a long because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long interval = 60000;  

void setup() {
  // initialize both serial ports:
  Serial.begin(115200);
  dht.begin();
  Serial1.begin(115200);

  delay(1000);

  sendData();
}

void loop() {
  // read from port 1, send to port 0:
  while (Serial1.available()) {
    int inByte = Serial1.read();
    Serial.write(inByte);
  }

   unsigned long currentMillis = millis();
 
  if(currentMillis - previousMillis > interval) {
    // save the last time you send the sensordata
    previousMillis = currentMillis;   
 
    sendData();
  }
}

void sendData(){
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius
  float t = dht.readTemperature();
  delay(2000);
  
  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  
  Serial.print(t,1);
  Serial.print(":");
  Serial.print(h,1);
  Serial.println(":4.603:52.506");
  Serial1.print(t,1);
  Serial1.print(":");
  Serial1.print(h,1);
  Serial1.println(":4.603:52.506");
}
