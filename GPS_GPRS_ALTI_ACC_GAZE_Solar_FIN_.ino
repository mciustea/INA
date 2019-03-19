//Jeremy Blum's Arduino Tutorial Series - Episode 15 - GPS Tracking
//Sample Code 2 - Logging GPS Data to an SD Card
//http://www.jeremyblum.com
//TinyGPS Library and Helper Functions by Mikal Hart http://arduiniana.org/libraries/tinygps/


#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include <String.h>
#include <SD.h>
#include <stdlib.h>
#include <Wire.h>
#include "IntersemaBaro.h"

Intersema::BaroPressure_MS5607B baro(true);

#define SENSOR_ADDR_OFF  (0x1D)//(Acc)
#define SENSOR_ADDR_ON   (0x53)

const uint8_t sensorAddr = SENSOR_ADDR_OFF;//ACC

// Sensor register addresses (gotten from datasheet)
#define REG_DEVID_ADDR        (0x00)
#define REG_THRESH_TAP_ADDR   (0x1d)
#define REG_TAP_DUR_ADDR      (0x21)
#define REG_TAP_LATENCY_ADDR  (0x22)
#define REG_TAP_WINDOW_ADDR   (0x23)      
#define REG_BW_RATE_ADDR      (0x2c)
#define REG_PWR_CTL_ADDR      (0x2d)
#define REG_INT_ENABLE_ADDR   (0x2e)
#define REG_DATA_FORMAT_ADDR  (0x31)
#define REG_DATAX0_ADDR       (0x32)
#define REG_DATAX1_ADDR       (0x33)
#define REG_DATAY0_ADDR       (0x34)
#define REG_DATAY1_ADDR       (0x35)
#define REG_DATAZ0_ADDR       (0x36)
#define REG_DATAZ1_ADDR       (0x37)
#define REG_FIFO_CTL_ADDR     (0x38)

/* GPS
  This sample code demonstrates the normal use of a TinyGPS object.
   It uses an Arduino Mega with a GPS attached to nss at 4800 buad.
*/

TinyGPS gps;
SoftwareSerial mySerial(7, 8);
SoftwareSerial nss(12, 255);
static char dtostrfbuffer[20];
int CS = 10;
int LED = 1;
int O3;
int Co2;
int sensorValueS;

//Define String
String SD_date_time = "invalid";
String SD_lat = "invalid";
String SD_lon = "invalid";
String dataString ="";

static void gpsdump(TinyGPS &gps);
static bool feedgps();
static void print_float(float val, float invalid, int len, int prec, int SD_val);
static void print_int(unsigned long val, unsigned long invalid, int len);
static void print_date(TinyGPS &gps);
static void print_str(const char *str, int len);

void setup()
{
  delay(500);
  pinMode(CS, OUTPUT);  //Chip Select Pin for the SD Card
  pinMode(LED, OUTPUT);  //LED Indicator
  pinMode(O3, INPUT);
  pinMode(Co2, INPUT);
  
  //Serial interfaces
  Serial.begin(9600);
  baro.init();
  mySerial.begin(19200);
  nss.begin(4800);
   Wire.begin();
   // Set 25 Hz output data rate and 25 Hz bandwidth and disable low power mode
   WriteByte(sensorAddr, REG_BW_RATE_ADDR, 0x08);

   // Disable auto sleep
   WriteByte(sensorAddr, REG_PWR_CTL_ADDR, 0x08);

   // Disable interrupts (the pins are not brought out anyway)
   WriteByte(sensorAddr, REG_INT_ENABLE_ADDR, 0x0);
  
  //Connect to the SD Card
  if(!SD.begin(CS))
  {
    Serial.println("Card Failure");
    return;
  }
  
  
  Serial.print("Testing TinyGPS library v. "); Serial.println(TinyGPS::library_version());
  Serial.println("by Mikal Hart");
  Serial.println();
  Serial.print("Sizeof(gpsobject) = "); Serial.println(sizeof(TinyGPS));
  Serial.println();
  Serial.println("Sats HDOP Latitude Longitude Fix  Date       Time       Date Alt     Course Speed Card  Distance Course Card  Chars Sentences Checksum");
  Serial.println("          (deg)    (deg)     Age                        Age  (m)     --- from GPS ----  --- to Aspendel  ---  RX    RX        Fail");
  Serial.println("--------------------------------------------------------------------------------------------------------------------------------------");
powerUpOrDown();
}



void loop()
{
 
  bool newdata = false;
  unsigned long start = millis();
  
   long alt = baro.getHeightCentiMeters();
   long altbun = alt / 100 ; 
  //Serial.print("Meters: ");
  //Serial.println((float)((altbun)))  ;
 // Serial.print(", Feet: ");
 // Serial.println((float)(alt) / 30.48);
   
  int temp = baro.getTemperature(); 
  //Serial.print("Temperature: "); 
  //Serial.println(temp);
  delay(100);
  int altG = gps.f_altitude(); 
  String dataStringALT = "Meters: " + String(altbun);
  String dataStringTEMP = "Temperature: " + String(temp);
  String dataStringALTG = "Meters(GPS): " + String(altG);
  
  //ACC->
  uint8_t devId;
   uint8_t x_msb;   // X-axis most significant byte
   uint8_t x_lsb;   // X-axis least significant byte
   uint8_t y_msb;   // Y-axis most significant byte
   uint8_t y_lsb;   // Y-axis least significant byte
   uint8_t z_msb;   // Z-axis most significant byte
   uint8_t z_lsb;   // Z-axis least significant byte
   uint16_t x;
   uint16_t y;
   uint16_t z;
   int xlescu;
   int ylescu;
   int zlescu;
   
   if (ReadByte(sensorAddr, 0x0, &devId) != 0)
   {
      Serial.println("Cannot read device ID from sensor");
   }
   else if (devId != 0xE5)
   {
      Serial.print("Wrong/invalid device ID ");
      Serial.print(devId);
      Serial.println(" (expected 0xE5)");
   }
   else
   {
      // Read the output
      if ((ReadByte(sensorAddr, REG_DATAX1_ADDR, &x_msb) == 0) &&
          (ReadByte(sensorAddr, REG_DATAX0_ADDR, &x_lsb) == 0) &&
          (ReadByte(sensorAddr, REG_DATAY1_ADDR, &y_msb) == 0) &&
          (ReadByte(sensorAddr, REG_DATAY0_ADDR, &y_lsb) == 0) &&
          (ReadByte(sensorAddr, REG_DATAZ1_ADDR, &z_msb) == 0) &&
          (ReadByte(sensorAddr, REG_DATAZ0_ADDR, &z_lsb) == 0))
      {
         x = (x_msb << 8) | x_lsb;
         y = (y_msb << 8) | y_lsb;
         z = (z_msb << 8) | z_lsb;

         // Perform 2's complement
         int16_t real_x = ~(x - 1);
         int16_t real_y = ~(y - 1);
         int16_t real_z = ~(z - 1);

         //Serial.print("X: ");
         //Serial.print(real_x);
         //Serial.print("   ");
         //Serial.print("  Y: ");
         //Serial.print(real_y);
         //Serial.print("   ");
         //Serial.print("  Z: ");
         //Serial.println(real_z);
         xlescu = real_x;
         ylescu = real_y;
         zlescu = real_z;
      }
      else
      {
         Serial.println("Failed to read from sensor");
      }

   }
   String dataStringx = " X: " + String(xlescu);
String dataStringy = " Y: " + String(ylescu);
String dataStringz = " Z: " + String(zlescu);
String dataStringTOT = dataStringx + "," + dataStringy + "," + dataStringz;
   // <=ACC
   
  
  // Every second we print an update
  while (millis() - start < 1000)
  {
    
    if (feedgps())
      newdata = true;
  }
  SenzorLumina();
  SenzorCo2();
  SenzorO3();
  String Codoi = " Co2: " + String(Co2);
  String Otrei = " O3: " + String(O3);
  String Gaze = Codoi + "," + Otrei;
  String Alimentare = "Solar Panel Level: " + String(sensorValueS);
  
  gpsdump(gps);
  
  //Write the newest information to the SD Card
  dataString = SD_date_time + "," + SD_lat + "," + SD_lon + "," + dataStringALT + "," + dataStringTEMP + "," + dataStringALTG + "," + dataStringTOT + "," + Gaze + "," + Alimentare;
  if(SD_date_time != "invalid")
    digitalWrite(LED, HIGH);
  else
    digitalWrite(LED, LOW);
    
  //Open the Data CSV File  
  File dataFile = SD.open("LOG2.csv", FILE_WRITE);
  if (dataFile)
  {
    dataFile.println(dataString);
    Serial.println("====================================================================================================================================");
    Serial.println("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++DATASTRING++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
    Serial.println(dataString);
    //Serial.println(gps.f_altitude());
    Serial.println("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++/DATASTRING++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
    Serial.println("====================================================================================================================================");
    dataFile.close();
  }
  else
  {
    Serial.println("\nCouldn't open the log file!");
  }
  SendTextMessage();
  
}

static void gpsdump(TinyGPS &gps)
{
  float flat, flon, falt;
  unsigned long age, date, time, chars = 0;
  unsigned short sentences = 0, failed = 0;
  static const float LONDON_LAT = 37.237743, LONDON_LON = -118.597591;
  
  print_int(gps.satellites(), TinyGPS::GPS_INVALID_SATELLITES, 5);
  print_int(gps.hdop(), TinyGPS::GPS_INVALID_HDOP, 5);
  gps.f_get_position(&flat, &flon, &age); 
  print_float(flat, TinyGPS::GPS_INVALID_F_ANGLE, 9, 5, 1); //LATITUDE
  print_float(flon, TinyGPS::GPS_INVALID_F_ANGLE, 10, 5, 2); //LONGITUDE
  print_int(age, TinyGPS::GPS_INVALID_AGE, 5);

  print_date(gps); //DATE AND TIME

  print_float(gps.f_altitude(), TinyGPS::GPS_INVALID_F_ALTITUDE, 8, 2, 0);
  print_float(gps.f_course(), TinyGPS::GPS_INVALID_F_ANGLE, 7, 2, 0);
  print_float(gps.f_speed_kmph(), TinyGPS::GPS_INVALID_F_SPEED, 6, 2, 0);
  print_str(gps.f_course() == TinyGPS::GPS_INVALID_F_ANGLE ? "*** " : TinyGPS::cardinal(gps.f_course()), 6);
  print_int(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0UL : (unsigned long)TinyGPS::distance_between(flat, flon, LONDON_LAT, LONDON_LON) / 1000, 0xFFFFFFFF, 9);
  print_float(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : TinyGPS::course_to(flat, flon, 51.508131, -0.128002), TinyGPS::GPS_INVALID_F_ANGLE, 7, 2, 0);
  print_str(flat == TinyGPS::GPS_INVALID_F_ANGLE ? "*** " : TinyGPS::cardinal(TinyGPS::course_to(flat, flon, LONDON_LAT, LONDON_LON)), 6);

  gps.stats(&chars, &sentences, &failed);
  print_int(chars, 0xFFFFFFFF, 6);
  print_int(sentences, 0xFFFFFFFF, 10);
  print_int(failed, 0xFFFFFFFF, 9);
  Serial.println();
}

static void print_int(unsigned long val, unsigned long invalid, int len)
{
  char sz[32];
  if (val == invalid)
    strcpy(sz, "*******");
  else
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  feedgps();
}

static void print_float(float val, float invalid, int len, int prec, int SD_val)
{
  char sz[32];
  if (val == invalid)
  {
    strcpy(sz, "*******");
    sz[len] = 0;
        if (len > 0) 
          sz[len-1] = ' ';
    for (int i=7; i<len; ++i)
        sz[i] = ' ';
    Serial.print(sz);
    if(SD_val == 1) SD_lat = sz;
    else if(SD_val == 2) SD_lon = sz;
  }
  else
  {
    Serial.print(val, prec);
    if (SD_val == 1) SD_lat = dtostrf(val,10,5,dtostrfbuffer);
    else if (SD_val == 2) SD_lon = dtostrf(val,10,5,dtostrfbuffer);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1);
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(" ");
  }
  feedgps();
}

static void print_date(TinyGPS &gps)
{
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned long age;
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  if (age == TinyGPS::GPS_INVALID_AGE)
  {
    Serial.print("*******    *******    ");
    SD_date_time = "invalid";
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d %02d:%02d:%02d   ",
        month, day, year, hour + 16, minute, second);
    Serial.print(sz);
    SD_date_time = sz;
  }
  print_int(age, TinyGPS::GPS_INVALID_AGE, 5);
  feedgps();
}

static void print_str(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  feedgps();
}

static bool feedgps()
{
  while (nss.available())
  {
    if (gps.encode(nss.read()))
      return true;
  }
  return false;
}

void powerUpOrDown()
{
  pinMode(9, OUTPUT); 
  digitalWrite(9,LOW);
  delay(1000);
  digitalWrite(9,HIGH);
  delay(2000);
  digitalWrite(9,LOW);
  delay(3000);
}

void SendTextMessage()
{
  
  mySerial.print("AT+CMGF=1\r");    //Because we want to send the SMS in text mode
  delay(100);
  mySerial.println("AT + CMGS = \"+14256358414\"");//send sms message, be careful need to add a country code before the cellphone number
  delay(100);
  mySerial.println(dataString);//the content of the message
  delay(100);
  mySerial.println((char)26);//the ASCII code of the ctrl+z is 26
  delay(100);
  mySerial.println();
}

int ReadByte(uint8_t addr, uint8_t reg, uint8_t *data)
{
   // Do an i2c write to set the register that we want to read from
   Wire.beginTransmission(addr);
   Wire.write(reg);
   Wire.endTransmission();

   // Read a byte from the device
   Wire.requestFrom(addr, (uint8_t)1);
   if (Wire.available())
   {
      *data = Wire.read();
   }
   else
   {
      // Read nothing back
      return -1;
   }

   return 0;
}

// Write a byte on the i2c interface
void WriteByte(uint8_t addr, uint8_t reg, byte data)
{
   // Begin the write sequence
   Wire.beginTransmission(addr);

   // First byte is to set the register pointer
   Wire.write(reg);

   // Write the data byte
   Wire.write(data);

   // End the write sequence; bytes are actually transmitted now
   Wire.endTransmission();
}
void SenzorO3()
{
O3 = analogRead(10);       // read analog input pin 0
  //Serial.println(O3, DEC);  // prints the value read
  //delay(100);     
}
void SenzorCo2()
{
Co2 = analogRead(11);       // read analog input pin 0
  //Serial.println(Co2, DEC);  // prints the value read
  //delay(100);     
}
void SenzorLumina()
{
  //Panouri Solare 
  sensorValueS = analogRead(12);       // read analog input pin 0
  //Serial.println(sensorValueS, DEC);  // prints the value read
  //delay(20);                        // wait 100ms for next reading
}
