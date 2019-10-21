#include <MemoryFree.h>

/*
  Arduino weerstation voor MCC Solar Voyager
  Verstuurt luchtvochtigheid en temperatuur naar een WiFi-module die het doorstuurt naar de database.
*/

#include <Time.h>
#include <TimeLib.h>

// include the SD library:
#include <SPI.h>
#include <SD.h>
// include the dht library:
#include <DHT.h>
#define DHTPIN 2                // what pin we're connected to with the sensor
#define DHTTYPE DHT22           // type sensor

DHT dht(DHTPIN, DHTTYPE);

const int chipSelect = 53;      // pin for SD-card

File dataFile, tempFile;        // files on SD-card, tempFile copies the whole dataFile except for the first line

long previousMillis = 0;        // will store last time voyager was updated
 
// the follow variables is a long because the time, measured in miliseconds,
// will quickly become a bigger number than cant be stored in an int.
long interval = 60000;  

bool readGPS = true;
bool messageSend = false;
bool sendSuccesful = false;
bool firstLine;

unsigned long currentMillis;

String succes = "   HTTP/1.1 200";
String timeGPS, dateGPS, dataString, line, data;
int hr,mth,dy,sec,mt,yr, coorDegree;
float h,t;
float latGPS, lonGPS, latGPScor, lonGPScor;
float divider = 100;
float coorMin, coor; 
String valid;
int count = 1;
char c;
String oldestData, copyString;

// Initial position hardcoded
float lat = 52.506;
float lon = 4.603;

void setup() {
  // initialize both serial ports:
  delay(2000);
  Serial.begin(9600);
  while(!Serial);
  dht.begin();
  Serial1.begin(9600);
  while(!Serial1);
  Serial2.begin(9600);
  while (!Serial2);

  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");
}

void loop() {
  // read from port 1, send to port 0:
  if(messageSend){
    while (Serial1.available()) {
      line = Serial1.readStringUntil('\n');
      line.trim();
      Serial.println(line);
      if(line == succes){
        Serial.println("200 message detected!");
        sendSuccesful = true;
      }
    }
  }

  if(sendSuccesful){
    Serial.println("Data succesfully posted to database!");
    messageSend = false;
    updateLogger();
  }

   while(Serial1.available()){
        Serial.write(Serial1.read());
  }

  // check if we need to read serial2 for gps data
  while(Serial2.available() && count < 1){
   data = Serial2.readStringUntil(',');
      // only go further if we got the right data
      if(data.equals("$GNRMC"))
      {
        count = 1;
        timeGPS = Serial2.readStringUntil(',');
        valid = Serial2.readStringUntil(',');
        latGPS = Serial2.readStringUntil(',').toFloat();
        Serial2.readStringUntil(',');
        lonGPS = Serial2.readStringUntil(',').toFloat();
        Serial2.readStringUntil(',');
        Serial2.readStringUntil(',');
        Serial2.readStringUntil(',');
        dateGPS = Serial2.readStringUntil(',');
        Serial.println("GPS data received correctly");
        return;
      }
      // Else we empty the serial buffer
      else{
        Serial.println("GPS compromised, trying again...");
        while(Serial2.available()){
          Serial2.read();
        }
      }
  }
  
  currentMillis = millis();

  // Only read gps 15 seconds before sending data
  if(readGPS){
    if(currentMillis - previousMillis > 45000) {      
      while(Serial2.available()){
        Serial2.read();
      }
      count = 0;
      readGPS = false;
    }
  }
  
  if(messageSend){
      if(currentMillis - previousMillis > 40000) {
        Serial.println("Send and receive window over.");
        messageSend = false;
    }
  }

  // send data after each minute
  if(currentMillis - previousMillis > interval) {
    // save the last time you send the sensordata
    previousMillis = currentMillis;   
    readGPS = true;
    logData();
  }
}

void logData(){
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  h = dht.readHumidity();
  // Read temperature as Celsius
  t = dht.readTemperature();
  delay(300);
  
  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  Serial.println("" + dateGPS + "" + timeGPS + "" + String(latGPS) + " " + String(lonGPS));
  latGPScor = coordinateConversion(latGPS);
  lonGPScor = coordinateConversion(lonGPS);
  if(valid.equals("A"))
  {
      lat = latGPScor;
      lon = lonGPScor;
  }
  if(timeGPS.length() > 5){
    // "time":"2019-10-03T11:46:20.904Z"
      // date from gps example : 151019
      // time from gps example : 124024.00
      hr = (timeGPS.substring(0,2)).toInt();
      mt = (timeGPS.substring(2,4)).toInt();
      sec = (timeGPS.substring(4,6)).toInt();
      yr = ("20" + dateGPS.substring(4,6)).toInt();;
      mth = (dateGPS.substring(2,4)).toInt();
      dy = (dateGPS.substring(0,2)).toInt();
      setTime(hr,mt,sec,dy,mth,yr);
      adjustTime(7200);
  }else{
    adjustTime(60);
    Serial.println("No GPS signal, latest latitude and longitude taken or waiting for first correct measurement");
  }
    
  //Serial.println("DateTime: " + dateGPS + "" + timeGPS + " LatGPS: " + String(latGPScor,3) + " Valid: " + valid + " LonGPS: " + String(lonGPScor,3));
  if(year() > 2018){
    dataString = "";
    dataString += String(year()) + "-" + String(month()) + "-" + String(day()) + "T" + String(hour()) + ":" + String(minute()) + ":00.000Z";
    dataString += ":" + String(t,1) + ":" + String(h,1) + ":" + String(lon,3) + ":" + String(lat,3);
  
    // open the log file. 
    dataFile = SD.open("datalog.txt", FILE_WRITE);
    delay(100);
  
    // if the file is available, write to it:
    if (dataFile) {
      dataFile.println(dataString);
      dataFile.close();
      // print to the serial port too:
      Serial.println("logged" + dataString);
      delay(250);
      sendData();
    }
    // if the file isn't open, pop up an error:
    else {
      Serial.println("error opening datalog.txt");
    }
  }
}

void sendData(){
  dataFile = SD.open("datalog.txt");
  delay(100);
  
  if (dataFile){
    firstLine = true;
    while (dataFile.available()) {
      copyString = dataFile.readStringUntil('\n');
      copyString.trim();
      if (firstLine) {
        oldestData = copyString;
          if(oldestData.length() > 2){
            messageSend = true;
            Serial.println("Data sending: " + oldestData);
            Serial1.println(oldestData);
          }
          firstLine = false;
        }
      }
       dataFile.close();
       delay(250);
  }else {
  // if the file didn't open, print an error:
  Serial.println("error opening datalog.txt for sending or nothing to send");
  messageSend = false;
  }  
}

void updateLogger(){
  Serial.println("Updating logfile"); 
  // first delete temp.txt file to get a new clean file
  SD.remove("temp.txt");
  delay(250);
  
  // open the files for reading writing:
  tempFile = SD.open("temp.txt", FILE_WRITE);
  delay(100);
  dataFile = SD.open("datalog.txt");
  delay(100);

  // copy datalog.txt to temp.txt except first line
  if (dataFile && tempFile) {
    firstLine = true;
    while (dataFile.available()) {
      copyString = dataFile.readStringUntil('\n');
      copyString.trim();
      if (firstLine) {
          firstLine = false;
        }else {
          if(copyString.length() > 0){
            tempFile.println(copyString);
          }else{
            Serial.println("Logger empty.");
          }
      }
    }
    tempFile.close();
    delay(100);
    dataFile.close();
    delay(100);
  }else {
    // if the file didn't open, print an error:
    Serial.println("error opening datalog- and temp.txt for copying");
  }
    
  // now copying temp to datalog, so first delete datalog.txt file to get a new clean file
  SD.remove("datalog.txt");
  delay(250);
    
  dataFile = SD.open("datalog.txt", FILE_WRITE);
  delay(100);
  tempFile = SD.open("temp.txt");
  delay(100);
  if (dataFile && tempFile) {
    if(tempFile.size() > 2){
      while (tempFile.available()) {
        copyString = tempFile.readStringUntil('\n');
        copyString.trim();
        dataFile.println(copyString);
      } 
      dataFile.close();
      delay(100);
      tempFile.close();
      delay(100);
      // set sendSuccesful to false to avoid emptying the log file.
      sendSuccesful = false;
      // previous message was delivered succesful, so try again to empty log file
      Serial.println(freeMemory());
      sendData();
    }else{
        Serial.println("Logger emptied");
    }
  }else {
  // if the file didn't open, print an error:
  Serial.println("error opening datalog.txt for sending");
  }  
  // set sendSuccesful to false to avoid emptying the log file.
    sendSuccesful = false;
}

// Coordinates need to be recalculate for real life latitudes and longitudes
float coordinateConversion(float coordinate){
  
  coorDegree = floor(coordinate / divider);
  coorMin = fmod(coordinate,divider);
  coor = float(coorDegree) + (coorMin / 60);

  return coor;
}
