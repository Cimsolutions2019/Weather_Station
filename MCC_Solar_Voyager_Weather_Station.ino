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
#define DHTPIN 48                // what pin we're connected to with the sensor
#define DHTTYPE DHT22           // type sensor

DHT dht(DHTPIN, DHTTYPE);

void(* resetFunc) (void) = 0;

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

char dataPrint[150];

String succes = "   HTTP/1.1 200";
String timeGPS, dateGPS, line, data, dataString, oldestData, copyString, valid;
int hr,mth,dy,sec,mt,yr, coorDegree;
float h, t, latGPS, lonGPS, latGPScor, lonGPScor, coorMin, coor;
float divider = 100;
int count = 1;
int resetCount = 0;
char c, serial1Read;

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

  sprintf(dataPrint, "Initializing SD card...");
  Serial.print(dataPrint);
  //Serial.print(F("Initializing SD card..."));

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    sprintf(dataPrint, "Card failed, or not present");
    Serial.println(dataPrint);
    //Serial.println(F("Card failed, or not present"));
    // don't do anything more:
    while (1);
  }
  sprintf(dataPrint, "card initialized.");
  Serial.println(dataPrint);
  //Serial.println(F("card initialized."));
}

void loop() {
  // read from port 1, send to port 0:
  if(messageSend){
    while (Serial1.available()) {
      line = Serial1.readStringUntil('\n');
      line.trim();
      if(line == succes){
        Serial.println(line);
        sendSuccesful = true;
      }
    }
  }

  cli();
  while(Serial1.available()){
    serial1Read = Serial1.read();
    Serial.write(serial1Read);
  }
  sei();

  if(sendSuccesful){
    messageSend = false;
    updateLogger();
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
        Serial.println(F("GPS data received correctly"));
        return;
      }
      // Else we empty the serial buffer
      else{
        Serial.println(F("GPS compromised, trying again"));
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
        Serial.println(F("Send-receive window over"));
        messageSend = false;
    }
  }

  // send data after each minute
  if(currentMillis - previousMillis > interval) {
    // save the last time you send the sensordata
    previousMillis = currentMillis;   
    readGPS = true;
    resetCount++;
    logData();
  }

  if(resetCount == 60){
    resetFunc();   // Te gebruiken als memory probleem niet opgelost kan worden.
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
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }
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
    Serial.println(F("No GPS signal, latest latitude and longitude taken or waiting for first correct measurement"));
  }
  //if(year() > 2018){
   
    dataString = "" + String(year()) + "-" + String(month()) + "-" + String(day()) + "T" + String(hour()) + ":" + String(minute()) + ":00.000Z" + ":" + String(t,1) + ":" + String(h,1) + ":" + String(lon,3) + ":" + String(lat,3);
    
    //Serial.println("Free ram: " + String(freeRam()));
    Serial.println(freeRam());
  
    // open the log file. 
    dataFile = SD.open("datalog.txt", FILE_WRITE);
    delay(100);
  
    // if the file is available, write to it:
    if (dataFile) {
      dataFile.println(dataString);
      dataFile.close();
      // print to the serial port too:
      Serial.println(dataString);
      delay(250);
      sendData();
    }
    // if the file isn't open, pop up an error:
    else {
      Serial.println(F("error opening datalog.txt"));
    }
  //}
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
  Serial.println(F("error opening datalog.txt for sending or nothing to send"));
  messageSend = false;
  }  
}

void updateLogger(){
  Serial.println(F("Updating logfile")); 
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
            Serial.println(F("Logger empty."));
          }
      }
    }
    tempFile.close();
    delay(100);
    dataFile.close();
    delay(100);
  }else {
    // if the file didn't open, print an error:
    Serial.println(F("error opening datalog- and temp.txt for copying"));
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
      sendData();
    }else{
        Serial.println(F("Logger emptied"));
    }
  }else {
  // if the file didn't open, print an error:
  Serial.println(F("error opening datalog.txt for sending"));
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

/* This function places the current value of the heap and stack pointers in the
 * variables. You can call it from any place in your code and save the data for
 * outputting or displaying later. This allows you to check at different parts of
 * your program flow.
 * The stack pointer starts at the top of RAM and grows downwards. The heap pointer
 * starts just above the static variables etc. and grows upwards. SP should always
 * be larger than HP or you'll be in big trouble! The smaller the gap, the more
 * careful you need to be. Julian Gall 6-Feb-2009.
 */

int freeRam() {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}
