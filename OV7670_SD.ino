// Pins configurations:
// SDA/SIOD ---> pin A4 (for Arduino UNO) | pin 20/SDA (for Arduino MEGA)
// SCL/SIOC ---> pin A5 (for Arduino UNO) | pin 21/SCL (for Arduino MEGA)
// MCLK/XCLK --> pin 11 (for Arduino UNO) | pin 10 (for Arduino MEGA)
// PCLK -------> pin 2
// VS/VSYNC ---> pin 3
// HS/HREF ----> pin 8
// D0 ---------> pin A0
// D1 ---------> pin A1
// D2 ---------> pin A2
// D3 ---------> pin A3
// D4 ---------> pin 4
// D5 ---------> pin 5
// D6 ---------> pin 6
// D7 ---------> pin 7

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Process.h>

File imageFile, raw1File, raw2File, raw3File, raw4File;

const int chipSelect = 53;      // pin for SD-card

int counter = 0;

#define CAMERA_ADDRESS 0x21

// Definitions of functions for manipulating the Arduino boards pins according to each Arduino board registers, so the code will work for both Arduino UNO and Arduino MEGA:
// The only change is the connections of the SDA/SIOD, SCL/SIOC and MCLK/XCLK pins to each board (see the pins configurations above).
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) // If you are using Arduino MEGA the IDE will automatically define the "__AVR_ATmega1280__" or "__AVR_ATmega2560__" constants.
#define TIMER2_PWM_A_PIN_MODE_OUTPUT() ({ DDRB |= 0b00010000; })
#define PIN2_DIGITAL_READ() ({ (PINE & 0b00010000) == 0 ? LOW : HIGH; })
#define PIN3_DIGITAL_READ() ({ (PINE & 0b00100000) == 0 ? LOW : HIGH; })
#define PIN4_DIGITAL_READ() ({ (PING & 0b00100000) == 0 ? LOW : HIGH; })
#define PIN5_DIGITAL_READ() ({ (PINE & 0b00001000) == 0 ? LOW : HIGH; })
#define PIN6_DIGITAL_READ() ({ (PINH & 0b00001000) == 0 ? LOW : HIGH; })
#define PIN7_DIGITAL_READ() ({ (PINH & 0b00010000) == 0 ? LOW : HIGH; })
#define PIN8_DIGITAL_READ() ({ (PINH & 0b00100000) == 0 ? LOW : HIGH; })
#define PINA0_DIGITAL_READ() ({ (PINF & 0b00000001) == 0 ? LOW : HIGH; })
#define PINA1_DIGITAL_READ() ({ (PINF & 0b00000010) == 0 ? LOW : HIGH; })
#define PINA2_DIGITAL_READ() ({ (PINF & 0b00000100) == 0 ? LOW : HIGH; })
#define PINA3_DIGITAL_READ() ({ (PINF & 0b00001000) == 0 ? LOW : HIGH; })
#elif defined(__AVR_ATmega328P__) // If you are using Arduino UNO the IDE will automatically define the "__AVR_ATmega328P__" constant.
#define TIMER2_PWM_A_PIN_MODE_OUTPUT() ({ DDRB |= 0b00001000; })
#define PIN2_DIGITAL_READ() ({ (PIND & 0b00000100) == 0 ? LOW : HIGH; })
#define PIN3_DIGITAL_READ() ({ (PIND & 0b00001000) == 0 ? LOW : HIGH; })
#define PIN4_DIGITAL_READ() ({ (PIND & 0b00010000) == 0 ? LOW : HIGH; })
#define PIN5_DIGITAL_READ() ({ (PIND & 0b00100000) == 0 ? LOW : HIGH; })
#define PIN6_DIGITAL_READ() ({ (PIND & 0b01000000) == 0 ? LOW : HIGH; })
#define PIN7_DIGITAL_READ() ({ (PIND & 0b10000000) == 0 ? LOW : HIGH; })
#define PIN8_DIGITAL_READ() ({ (PINB & 0b00000001) == 0 ? LOW : HIGH; })
#define PINA0_DIGITAL_READ() ({ (PINC & 0b00000001) == 0 ? LOW : HIGH; })
#define PINA1_DIGITAL_READ() ({ (PINC & 0b00000010) == 0 ? LOW : HIGH; })
#define PINA2_DIGITAL_READ() ({ (PINC & 0b00000100) == 0 ? LOW : HIGH; })
#define PINA3_DIGITAL_READ() ({ (PINC & 0b00001000) == 0 ? LOW : HIGH; })
#endif

void initializeCamera() {
  // these are common registers values for initializing the camera to QVGA (320 x 240) mode at the maximum clock prescaler:
  writeCameraRegister(0x12, 0x80); // reset all camera registers to default value
  delay(1000); // wait for reset proccess to be done (because it is quite slow)
  writeCameraRegister(0x3A, 0x04);
  writeCameraRegister(0x12, 0x00);
  writeCameraRegister(0x17, 0x13);
  writeCameraRegister(0x18, 0x01);
  writeCameraRegister(0x32, 0xB6);
  writeCameraRegister(0x19, 0x02);
  writeCameraRegister(0x1A, 0x7A);
  writeCameraRegister(0x03, 0x0A);
  writeCameraRegister(0x0C, 0x00);
  writeCameraRegister(0x3E, 0x00);
  writeCameraRegister(0x70, 0x3A);
  writeCameraRegister(0x71, 0x35);
  writeCameraRegister(0x72, 0x11);
  writeCameraRegister(0x73, 0xF0);
  writeCameraRegister(0xA2, 0x01);
  writeCameraRegister(0x15, 0x00);
  writeCameraRegister(0x7A, 0x20);
  writeCameraRegister(0x7B, 0x10);
  writeCameraRegister(0x7C, 0x1E);
  writeCameraRegister(0x7D, 0x35);
  writeCameraRegister(0x7E, 0x5A);
  writeCameraRegister(0x7F, 0x69);
  writeCameraRegister(0x80, 0x76);
  writeCameraRegister(0x81, 0x80);
  writeCameraRegister(0x82, 0x88);
  writeCameraRegister(0x83, 0x8F);
  writeCameraRegister(0x84, 0x96);
  writeCameraRegister(0x85, 0xA3);
  writeCameraRegister(0x86, 0xAF);
  writeCameraRegister(0x87, 0xC4);
  writeCameraRegister(0x88, 0xD7);
  writeCameraRegister(0x89, 0xE8);
  writeCameraRegister(0x13, 0xC0);
  writeCameraRegister(0x00, 0x00);
  writeCameraRegister(0x10, 0x00);
  writeCameraRegister(0x0D, 0x40);
  writeCameraRegister(0x14, 0x18);
  writeCameraRegister(0xA5, 0x05);
  writeCameraRegister(0xAB, 0x07);
  writeCameraRegister(0x24, 0x95);
  writeCameraRegister(0x25, 0x33);
  writeCameraRegister(0x26, 0xE3);
  writeCameraRegister(0x9F, 0x78);
  writeCameraRegister(0xA0, 0x68);
  writeCameraRegister(0xA1, 0x03);
  writeCameraRegister(0xA6, 0xD8);
  writeCameraRegister(0xA7, 0xD8);
  writeCameraRegister(0xA8, 0xF0);
  writeCameraRegister(0xA9, 0x90);
  writeCameraRegister(0xAA, 0x94);
  writeCameraRegister(0x13, 0xC5);
  writeCameraRegister(0x30, 0x00);
  writeCameraRegister(0x31, 0x00);
  writeCameraRegister(0x0E, 0x61);
  writeCameraRegister(0x0F, 0x4B);
  writeCameraRegister(0x16, 0x02);
  writeCameraRegister(0x1E, 0x07);
  writeCameraRegister(0x21, 0x02);
  writeCameraRegister(0x22, 0x91);
  writeCameraRegister(0x29, 0x07);
  writeCameraRegister(0x33, 0x0B);
  writeCameraRegister(0x35, 0x0B);
  writeCameraRegister(0x37, 0x1D);
  writeCameraRegister(0x38, 0x71);
  writeCameraRegister(0x39, 0x2A);
  writeCameraRegister(0x3C, 0x78);
  writeCameraRegister(0x4D, 0x40);
  writeCameraRegister(0x4E, 0x20);
  writeCameraRegister(0x69, 0x00);
  writeCameraRegister(0x74, 0x10);
  writeCameraRegister(0x8D, 0x4F);
  writeCameraRegister(0x8E, 0x00);
  writeCameraRegister(0x8F, 0x00);
  writeCameraRegister(0x90, 0x00);
  writeCameraRegister(0x91, 0x00);
  writeCameraRegister(0x96, 0x00);
  writeCameraRegister(0x9A, 0x00);
  writeCameraRegister(0xB0, 0x84);
  writeCameraRegister(0xB1, 0x0C);
  writeCameraRegister(0xB2, 0x0E);
  writeCameraRegister(0xB3, 0x82);
  writeCameraRegister(0xB8, 0x0A);
  writeCameraRegister(0x43, 0x0A);
  writeCameraRegister(0x44, 0xF0);
  writeCameraRegister(0x45, 0x34);
  writeCameraRegister(0x46, 0x58);
  writeCameraRegister(0x47, 0x28);
  writeCameraRegister(0x48, 0x3A);
  writeCameraRegister(0x59, 0x88);
  writeCameraRegister(0x5A, 0x88);
  writeCameraRegister(0x5B, 0x44);
  writeCameraRegister(0x5C, 0x67);
  writeCameraRegister(0x5D, 0x49);
  writeCameraRegister(0x5E, 0x0E);
  writeCameraRegister(0x6C, 0x0A);
  writeCameraRegister(0x6D, 0x55);
  writeCameraRegister(0x6E, 0x11);
  writeCameraRegister(0x6F, 0x9E);
  writeCameraRegister(0x6A, 0x40);
  writeCameraRegister(0x01, 0x40);
  writeCameraRegister(0x02, 0x60);
  writeCameraRegister(0x13, 0xC7);
  writeCameraRegister(0x4F, 0x80);
  writeCameraRegister(0x50, 0x80);
  writeCameraRegister(0x51, 0x00);
  writeCameraRegister(0x52, 0x22);
  writeCameraRegister(0x53, 0x5E);
  writeCameraRegister(0x54, 0x80);
  writeCameraRegister(0x58, 0x9E);
  writeCameraRegister(0x41, 0x08);
  writeCameraRegister(0x3F, 0x00);
  writeCameraRegister(0x75, 0x05);
  writeCameraRegister(0x76, 0xE1);
  writeCameraRegister(0x4C, 0x00);
  writeCameraRegister(0x77, 0x01);
  writeCameraRegister(0x3D, 0x48);
  writeCameraRegister(0x4B, 0x09);
  writeCameraRegister(0xC9, 0x60);
  writeCameraRegister(0x56, 0x40);
  writeCameraRegister(0x34, 0x11);
  writeCameraRegister(0x3B, 0x12);
  writeCameraRegister(0xA4, 0x82);
  writeCameraRegister(0x96, 0x00);
  writeCameraRegister(0x97, 0x30);
  writeCameraRegister(0x98, 0x20);
  writeCameraRegister(0x99, 0x30);
  writeCameraRegister(0x9A, 0x84);
  writeCameraRegister(0x9B, 0x29);
  writeCameraRegister(0x9C, 0x03);
  writeCameraRegister(0x9D, 0x4C);
  writeCameraRegister(0x9E, 0x3F);
  writeCameraRegister(0x78, 0x04);
  writeCameraRegister(0x79, 0x01);
  writeCameraRegister(0xC8, 0xF0);
  writeCameraRegister(0x79, 0x0F);
  writeCameraRegister(0xC8, 0x00);
  writeCameraRegister(0x79, 0x10);
  writeCameraRegister(0xC8, 0x7E);
  writeCameraRegister(0x79, 0x0A);
  writeCameraRegister(0xC8, 0x80);
  writeCameraRegister(0x79, 0x0B);
  writeCameraRegister(0xC8, 0x01);
  writeCameraRegister(0x79, 0x0C);
  writeCameraRegister(0xC8, 0x0F);
  writeCameraRegister(0x79, 0x0D);
  writeCameraRegister(0xC8, 0x20);
  writeCameraRegister(0x79, 0x09);
  writeCameraRegister(0xC8, 0x80);
  writeCameraRegister(0x79, 0x02);
  writeCameraRegister(0xC8, 0xC0);
  writeCameraRegister(0x79, 0x03);
  writeCameraRegister(0xC8, 0x40);
  writeCameraRegister(0x79, 0x05);
  writeCameraRegister(0xC8, 0x30);
  writeCameraRegister(0x79, 0x26);
  writeCameraRegister(0xFF, 0xFF);
  writeCameraRegister(0x15, 0x20);
  writeCameraRegister(0x0C, 0x04);
  writeCameraRegister(0x3E, 0x19);
  writeCameraRegister(0x72, 0x11);
  writeCameraRegister(0x73, 0xF1);
  writeCameraRegister(0x17, 0x16);
  writeCameraRegister(0x18, 0x04);
  writeCameraRegister(0x32, 0xA4);
  writeCameraRegister(0x19, 0x02);
  writeCameraRegister(0x1A, 0x7A);
  writeCameraRegister(0x03, 0x0A);
  writeCameraRegister(0xFF, 0xFF);
  writeCameraRegister(0x12, 0x00);
  writeCameraRegister(0x8C, 0x00);
  writeCameraRegister(0x04, 0x00);
  writeCameraRegister(0x40, 0xC0);
  writeCameraRegister(0x14, 0x6A);
  writeCameraRegister(0x4F, 0x80);
  writeCameraRegister(0x50, 0x80);
  writeCameraRegister(0x51, 0x00);
  writeCameraRegister(0x52, 0x22);
  writeCameraRegister(0x53, 0x5E);
  writeCameraRegister(0x54, 0x80);
  writeCameraRegister(0x3D, 0x40);
  writeCameraRegister(0xFF, 0xFF);
  writeCameraRegister(0x11, 0x1F);
  writeCameraRegister(0x6B, 0x00);
  writeCameraRegister(0x0C, 0x08);
  writeCameraRegister(0x3E, 0x19);
  writeCameraRegister(0x73, 0xF1);
  writeCameraRegister(0x12, 0x10);
  delay(3000); // wait for registers to be set (because it is quite slow)
}

void initializePWMTimer() {
  cli();
  TIMER2_PWM_A_PIN_MODE_OUTPUT(); // Set the A PWM pin of TIMER2 to output
  ASSR &= ~(_BV(EXCLK) | _BV(AS2));
  TCCR2A = (1 << COM2A0) | (1 << WGM21) | (1 << WGM20);
  TCCR2B = (1 << WGM22) | (1 << CS20);
  OCR2A = 0;
  sei();
}

byte readCameraRegister(byte registerId) {
  Wire.beginTransmission(CAMERA_ADDRESS);
  Wire.write(registerId);
  Wire.endTransmission();
  Wire.requestFrom(CAMERA_ADDRESS, 1);
  while (Wire.available() <= 0);
  byte registerValue = Wire.read();
  delay(1);
  return registerValue;
}

void writeCameraRegister(byte registerId, byte registerValue) {
  Wire.beginTransmission(CAMERA_ADDRESS);
  Wire.write(registerId);
  Wire.write(registerValue);
  Wire.endTransmission();
  delay(1);
}

void captureFrame(char* title) {
  Serial.println(title);

  SD.remove(title);
  SD.remove("part1.bmp");
  SD.remove("part2.bmp");
  SD.remove("part3.bmp");
  SD.remove("part4.bmp");
  const int w = 320;                   // image width in pixels
  const int h = 240;                    // " height
  byte dataBuffer[w];
  int check = 0;
  unsigned int n;
  delay(1000);
  
  raw1File = SD.open("part1.bmp", O_CREAT | O_WRITE);
  Serial.println(F("1"));
  
  unsigned int tempWidth = 0;
  unsigned int frameWidth = 320;
  unsigned int frameHeight = 240;
  cli(); // disable all interrupts during frame capture (because it needs to be as fast as possible)
  while (PIN3_DIGITAL_READ() == LOW); // wait until VS/VSYNC pin is high
  while (PIN3_DIGITAL_READ() == HIGH); // wait until VS/VSYNC pin is low
  while (frameHeight--) {
    tempWidth = frameWidth;
    check = frameHeight%4;
    while (tempWidth--) {
        while (PIN2_DIGITAL_READ() == LOW); // wait until PCLK pin is high
        while (PIN2_DIGITAL_READ() == HIGH); // wait until PCLK pin is low
      if( check == 3){
        byte byteToWrite = 0b00000000;
        byteToWrite |= ((PIN7_DIGITAL_READ() == HIGH) << 7);
        byteToWrite |= ((PIN6_DIGITAL_READ() == HIGH) << 6);
        byteToWrite |= ((PIN5_DIGITAL_READ() == HIGH) << 5);
        byteToWrite |= ((PIN4_DIGITAL_READ() == HIGH) << 4);
        byteToWrite |= ((PINA3_DIGITAL_READ() == HIGH) << 3);
        byteToWrite |= ((PINA2_DIGITAL_READ() == HIGH) << 2);
        byteToWrite |= ((PINA1_DIGITAL_READ() == HIGH) << 1);
        byteToWrite |= ((PINA0_DIGITAL_READ() == HIGH));
        //UDR0 = byteToWrite; // send data via serial connection with UART register (we need to use the serial register directly for fast transfer)
        dataBuffer[319-tempWidth] = byteToWrite;
      }
        while (PIN2_DIGITAL_READ() == LOW); // wait until PCLK pin is high
        while (PIN2_DIGITAL_READ() == HIGH); // wait until PCLK pin is low
        // ignore each second byte (for a grayscale image we only need each first byte, which represents luminescence)      
    }
    if( check == 3){
      raw1File.write(dataBuffer, 320);
    }
  }
  sei(); // enable all interrupts
  raw1File.close();
  
  raw2File = SD.open("part2.bmp", O_CREAT | O_WRITE);
  Serial.println(F("2"));

  tempWidth = 0;
  frameWidth = 320;
  frameHeight = 240;
  cli(); // disable all interrupts during frame capture (because it needs to be as fast as possible)
  while (PIN3_DIGITAL_READ() == LOW); // wait until VS/VSYNC pin is high
  while (PIN3_DIGITAL_READ() == HIGH); // wait until VS/VSYNC pin is low
  while (frameHeight--) {
    tempWidth = frameWidth;
    check = frameHeight%4;
    while (tempWidth--) {
        while (PIN2_DIGITAL_READ() == LOW); // wait until PCLK pin is high
        while (PIN2_DIGITAL_READ() == HIGH); // wait until PCLK pin is low
      if( check == 2){
        byte byteToWrite = 0b00000000;
        byteToWrite |= ((PIN7_DIGITAL_READ() == HIGH) << 7);
        byteToWrite |= ((PIN6_DIGITAL_READ() == HIGH) << 6);
        byteToWrite |= ((PIN5_DIGITAL_READ() == HIGH) << 5);
        byteToWrite |= ((PIN4_DIGITAL_READ() == HIGH) << 4);
        byteToWrite |= ((PINA3_DIGITAL_READ() == HIGH) << 3);
        byteToWrite |= ((PINA2_DIGITAL_READ() == HIGH) << 2);
        byteToWrite |= ((PINA1_DIGITAL_READ() == HIGH) << 1);
        byteToWrite |= ((PINA0_DIGITAL_READ() == HIGH));
        //UDR0 = byteToWrite; // send data via serial connection with UART register (we need to use the serial register directly for fast transfer)
        dataBuffer[319-tempWidth] = byteToWrite;
      }
        while (PIN2_DIGITAL_READ() == LOW); // wait until PCLK pin is high
        while (PIN2_DIGITAL_READ() == HIGH); // wait until PCLK pin is low
        // ignore each second byte (for a grayscale image we only need each first byte, which represents luminescence)      
    }
    if( check == 2){
      raw2File.write(dataBuffer, 320);
    }
  }
  sei(); // enable all interrupts
  raw2File.close();

  raw3File = SD.open("part3.bmp", O_CREAT | O_WRITE);
  Serial.println(F("3"));

  tempWidth = 0;
  frameWidth = 320;
  frameHeight = 240;
  cli(); // disable all interrupts during frame capture (because it needs to be as fast as possible)
  while (PIN3_DIGITAL_READ() == LOW); // wait until VS/VSYNC pin is high
  while (PIN3_DIGITAL_READ() == HIGH); // wait until VS/VSYNC pin is low
  while (frameHeight--) {
    tempWidth = frameWidth;
    check = frameHeight%4;
    while (tempWidth--) {
        while (PIN2_DIGITAL_READ() == LOW); // wait until PCLK pin is high
        while (PIN2_DIGITAL_READ() == HIGH); // wait until PCLK pin is low
      if( check == 1){
        byte byteToWrite = 0b00000000;
        byteToWrite |= ((PIN7_DIGITAL_READ() == HIGH) << 7);
        byteToWrite |= ((PIN6_DIGITAL_READ() == HIGH) << 6);
        byteToWrite |= ((PIN5_DIGITAL_READ() == HIGH) << 5);
        byteToWrite |= ((PIN4_DIGITAL_READ() == HIGH) << 4);
        byteToWrite |= ((PINA3_DIGITAL_READ() == HIGH) << 3);
        byteToWrite |= ((PINA2_DIGITAL_READ() == HIGH) << 2);
        byteToWrite |= ((PINA1_DIGITAL_READ() == HIGH) << 1);
        byteToWrite |= ((PINA0_DIGITAL_READ() == HIGH));
        //UDR0 = byteToWrite; // send data via serial connection with UART register (we need to use the serial register directly for fast transfer)
        dataBuffer[319-tempWidth] = byteToWrite;
      }
        while (PIN2_DIGITAL_READ() == LOW); // wait until PCLK pin is high.
        while (PIN2_DIGITAL_READ() == HIGH); // wait until PCLK pin is low
        // ignore each second byte (for a grayscale image we only need each first byte, which represents luminescence)      
    }
    if( check == 1){
      raw3File.write(dataBuffer, 320);
    }
  }
  sei(); // enable all interrupts
  raw3File.close();

  raw4File = SD.open("part4.bmp", O_CREAT | O_WRITE);
  Serial.println(F("4"));

  tempWidth = 0;
  frameWidth = 320;
  frameHeight = 240;
  cli(); // disable all interrupts during frame capture (because it needs to be as fast as possible)
  while (PIN3_DIGITAL_READ() == LOW); // wait until VS/VSYNC pin is high
  while (PIN3_DIGITAL_READ() == HIGH); // wait until VS/VSYNC pin is low
  while (frameHeight--) {
    tempWidth = frameWidth;
    check = frameHeight%4;
    while (tempWidth--) {
        while (PIN2_DIGITAL_READ() == LOW); // wait until PCLK pin is high
        while (PIN2_DIGITAL_READ() == HIGH); // wait until PCLK pin is low
      if( check == 0){
        byte byteToWrite = 0b00000000;
        byteToWrite |= ((PIN7_DIGITAL_READ() == HIGH) << 7);
        byteToWrite |= ((PIN6_DIGITAL_READ() == HIGH) << 6);
        byteToWrite |= ((PIN5_DIGITAL_READ() == HIGH) << 5);
        byteToWrite |= ((PIN4_DIGITAL_READ() == HIGH) << 4);
        byteToWrite |= ((PINA3_DIGITAL_READ() == HIGH) << 3);
        byteToWrite |= ((PINA2_DIGITAL_READ() == HIGH) << 2);
        byteToWrite |= ((PINA1_DIGITAL_READ() == HIGH) << 1);
        byteToWrite |= ((PINA0_DIGITAL_READ() == HIGH));
        //UDR0 = byteToWrite; // send data via serial connection with UART register (we need to use the serial register directly for fast transfer)
        dataBuffer[319-tempWidth] = byteToWrite;
      }
        while (PIN2_DIGITAL_READ() == LOW); // wait until PCLK pin is high
        while (PIN2_DIGITAL_READ() == HIGH); // wait until PCLK pin is low
        // ignore each second byte (for a grayscale image we only need each first byte, which represents luminescence)      
    }
    if( check == 0){
      raw4File.write(dataBuffer, 320);
    }
  }
  sei(); // enable all interrupts
  raw4File.close();
  delay(100);

  Serial.println(F("writing header"));
  
  const int imgSize = w*h;

  // set fileSize (used in bmp header)
  int rowSize = 4 * ((3*w + 3)/4);      // how many bytes in the row (used to create padding)
  int fileSize = 54 + h*rowSize;        // headers (54 bytes) + pixel data

  // create padding (based on the number of pixels in a row
  unsigned char bmpPad[rowSize - 3*w];
  for (int i=0; i<sizeof(bmpPad); i++) {         // fill with 0s
    bmpPad[i] = 0;
  }

  // create file headers (also taken from StackOverflow example)
  unsigned char bmpFileHeader[14] = {            // file header (always starts with BM!)
    'B','M', 0,0,0,0, 0,0, 0,0, 54,0,0,0   };
  unsigned char bmpInfoHeader[40] = {            // info about the file (size, etc)
    40,0,0,0, 0,0,0,0, 0,0,0,0, 1,0, 24,0   };

  bmpFileHeader[ 2] = (unsigned char)(fileSize      );
  bmpFileHeader[ 3] = (unsigned char)(fileSize >>  8);
  bmpFileHeader[ 4] = (unsigned char)(fileSize >> 16);
  bmpFileHeader[ 5] = (unsigned char)(fileSize >> 24);

  bmpInfoHeader[ 4] = (unsigned char)(       w      );
  bmpInfoHeader[ 5] = (unsigned char)(       w >>  8);
  bmpInfoHeader[ 6] = (unsigned char)(       w >> 16);
  bmpInfoHeader[ 7] = (unsigned char)(       w >> 24);
  bmpInfoHeader[ 8] = (unsigned char)(       h      );
  bmpInfoHeader[ 9] = (unsigned char)(       h >>  8);
  bmpInfoHeader[10] = (unsigned char)(       h >> 16);
  bmpInfoHeader[11] = (unsigned char)(       h >> 24);

  // write the file (thanks forum!)
  imageFile = SD.open(title, O_CREAT | O_WRITE);
  imageFile.write(bmpFileHeader, sizeof(bmpFileHeader));    // write file header
  imageFile.write(bmpInfoHeader, sizeof(bmpInfoHeader));    // " info header
  delay(100);
  
  Serial.println(F("writing bytes"));

  raw1File = SD.open("part1.bmp");
  raw2File = SD.open("part2.bmp");
  raw3File = SD.open("part3.bmp");
  raw4File = SD.open("part4.bmp");

  for(int k = 0; k < 60;k++){
    raw1File.read(dataBuffer, 320);
    for(int l = 0; l < 320;l++){
      byte triByte[3] = { dataBuffer[l], dataBuffer[l], dataBuffer[l] };
      imageFile.write(triByte,3);
    }
    imageFile.write(bmpPad, (4-(w*3)%4)%4); 
    raw2File.read(dataBuffer, 320);
    for(int l = 0; l < 320;l++){
      byte triByte[3] = { dataBuffer[l], dataBuffer[l], dataBuffer[l] };
      imageFile.write(triByte,3);
    }
    imageFile.write(bmpPad, (4-(w*3)%4)%4); 
    raw3File.read(dataBuffer, 320);
    for(int l = 0; l < 320;l++){
      byte triByte[3] = { dataBuffer[l], dataBuffer[l], dataBuffer[l] };
      imageFile.write(triByte,3);
    }
    imageFile.write(bmpPad, (4-(w*3)%4)%4); 
    raw4File.read(dataBuffer, 320);
    for(int l = 0; l < 320;l++){
      byte triByte[3] = { dataBuffer[l], dataBuffer[l], dataBuffer[l] };
      imageFile.write(triByte,3);
    }
    imageFile.write(bmpPad, (4-(w*3)%4)%4); 
    imageFile.flush();
  }
  raw1File.close();
  raw2File.close(); 
  raw3File.close();
  raw4File.close();
  imageFile.close();

//  for(int l = 0; l < 320;l++){
//    raw1File = SD.open("part1.bmp");
//    raw2File = SD.open("part2.bmp");
//    raw3File = SD.open("part3.bmp");
//    raw4File = SD.open("part4.bmp");
//    for(int k = 0; k < 60;k++){
//      byte triByteFourLines[12];
//      raw1File.read(dataBuffer, 320);
//      triByteFourLines[0]= dataBuffer[k];
//      triByteFourLines[1]= dataBuffer[k];
//      triByteFourLines[2]= dataBuffer[k];
//      raw2File.read(dataBuffer, 320);
//      triByteFourLines[3]= dataBuffer[k];
//      triByteFourLines[4]= dataBuffer[k];
//      triByteFourLines[5]= dataBuffer[k];
//      raw3File.read(dataBuffer, 320);
//      triByteFourLines[6]= dataBuffer[k];
//      triByteFourLines[7]= dataBuffer[k];
//      triByteFourLines[8]= dataBuffer[k];
//      raw4File.read(dataBuffer, 320);
//      triByteFourLines[9]= dataBuffer[k];
//      triByteFourLines[10]= dataBuffer[k];
//      triByteFourLines[11]= dataBuffer[k];
//      imageFile.write(triByteFourLines,12);
//    }
//    imageFile.write(bmpPad, (4-(w*3)%4)%4); 
//    imageFile.flush();
//    raw1File.close();
//    raw2File.close(); 
//    raw3File.close();
//    raw4File.close();
//  }
//  imageFile.close();
}

void setup() {
  initializePWMTimer();
  Wire.begin();
  Serial.begin(1000000); // the frame capture software communicates with the Arduino at a baud rate of 1MHz
  initializeCamera();

  Serial.print(F("Initializing SD card..."));
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println(F("Card failed, or not present"));
    // don't do anything more:
    while (1);
  }
  Serial.println(F("card initialized."));
}

void loop() {
  captureFrame("1.bmp"); 
  //  //merge header with raw data
//  imageFile = sd.open(title, O_WRITE | O_CREAT | O_TRUNC);
//  rawFile = sd.open("rawdata.txt");
//  imageFile.write(header, 54);
//  delay(100);
//
//  if (imageFile && rawFile) {
//    while (imageFile.available()) {
//      uint8_t raw = rawFile.read();
//      imageFile.write(raw);
//    }
//  }else {
//    // if the file didn't open, print an error:
//    Serial.println(F("error opening datalog- and temp.txt for copying"));
//  }
//  rawFile.close();
//  delay(10);
//  imageFile.close();
  captureFrame("2.bmp");
  captureFrame("3.bmp");
  captureFrame("4.bmp");
  captureFrame("5.bmp");
  captureFrame("6.bmp");
  captureFrame("7.bmp");
  captureFrame("8.bmp");
  captureFrame("9.bmp");
  captureFrame("10.bmp");
  captureFrame("11.bmp");
  captureFrame("12.bmp");
  captureFrame("13.bmp");
  captureFrame("14.bmp");
  captureFrame("15.bmp");
  captureFrame("16.bmp");
  Serial.write("done");
//  Process p;
//  Serial.println("Now converting .csv to functional .bmp");
//  p.begin("python");     
//  p.addParameter("csvToVec.py");
//  p.run();
  //p.runShellCommand("python csvToVec.py");
  while(1);
}
