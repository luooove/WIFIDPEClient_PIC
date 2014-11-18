/************************************************************************/
/*                                                                      */
/*      UDPEchoClient                                                   */
/*                                                                      */
/*      A chipKIT DNETcK UDP Client application to                      */
/*      demonstrate how to use the UdpClient Class.                     */
/*      This can be used in conjuction  with UDPEchoServer              */
/*                                                                      */
/************************************************************************/
/*      Author:       Keith Vogel                                       */
/*      Copyright 2011, Digilent Inc.                                   */
/************************************************************************/
/*
  This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */
/************************************************************************/
/*                                                                      */
/*                                                                      */
/************************************************************************/
/*  Revision History:                                                   */
/*                                                                      */
/*      12/21/2011(KeithV): Created                                     */
/*      2/3/2012(KeithV): Updated for WiFi                              */
/*      11/13/2012(KeithV): Modified to be generic for all HW libraries */
/*                                                                      */
/************************************************************************/

//******************************************************************************************
//******************************************************************************************
//***************************** SET YOUR CONFIGURATION *************************************
//******************************************************************************************
//******************************************************************************************

/*
2014年11月18日
Designer：骆晓祥 刘宇林 王衡
测量值：	传感器						        端口
温湿度		DTH11						数字口10
PM2.5		GP2Y1010AU0F 				        模拟口A0 数字口2
气压		BMP180						SCL 
液位		液位传感器（电容）			                模拟口A11
风速		光电开关	                                        数字口3
光线		光线传感器 模拟输出 			        模拟口A6
风向		光线传感器 模拟输出 			        模拟口A1				
*/




/************************************************************************/
/*                                                                      */
/*              Include ONLY 1 hardware library that matches            */
/*              the network hardware you are using                      */
/*                                                                      */
/*              Refer to the hardware library header file               */
/*              for supported boards and hardware configurations        */
/*                                                                      */
/************************************************************************/
 #include <string.h>
 #include <ctype.h>

// #include <WiFiShieldOrPmodWiFi.h>                       // This is for the MRF24WBxx on a pmodWiFi or WiFiShield
#include <WiFiShieldOrPmodWiFi_G.h>                     // This is for the MRF24WGxx on a pmodWiFi or WiFiShield

/************************************************************************/
/*                    Required libraries, Do NOT comment out            */
/************************************************************************/
#include <DNETcK.h>
#include <DWIFIcK.h>
#include "Wire.h"
#include "Adafruit_BMP085.h"

Adafruit_BMP085 bmp;

#include <SD.h>

File myFile;
// On the Ethernet Shield, CS is pin 4. It's set as an output by default.
// Note that even if it's not used as the CS pin, the hardware SS pin 
// (10 on most Arduino boards, 53 on the Mega) must be left as an output 
// or the SD library functions will not work. 

// Default SD chip select for Uno and Mega type devices
const int chipSelect_SD_default = 10; // Change 10 to 53 for a Mega

// chipSelect_SD can be changed if you do not use default CS pin
const int chipSelect_SD = chipSelect_SD_default;
/************************************************************************/
/*                                                                      */
/*              SET THESE VALUES FOR YOUR NETWORK                       */
/*                                                                      */
/************************************************************************/

char * szIPServer = "192.168.165.152";
unsigned short portServer = 8080;           //DNETcK::iPersonalPorts44 + 400;     // port 44400

// Specify the SSID
const char * szSsid = "robot";

// select 1 for the security you want, or none for no security
#define USE_WPA2_PASSPHRASE
//#define USE_WPA2_KEY
//#define USE_WEP40
//#define USE_WEP104
//#define USE_WF_CONFIG_H

// modify the security key to what you have.
#if defined(USE_WPA2_PASSPHRASE)

const char * szPassPhrase = "swjtumakerspace";
#define WiFiConnectMacro() DWIFIcK::connect(szSsid, szPassPhrase, &status)

#elif defined(USE_WPA2_KEY)

DWIFIcK::WPA2KEY key = { 
  0x27, 0x2C, 0x89, 0xCC, 0xE9, 0x56, 0x31, 0x1E, 
  0x3B, 0xAD, 0x79, 0xF7, 0x1D, 0xC4, 0xB9, 0x05, 
  0x7A, 0x34, 0x4C, 0x3E, 0xB5, 0xFA, 0x38, 0xC2, 
  0x0F, 0x0A, 0xB0, 0x90, 0xDC, 0x62, 0xAD, 0x58 };
#define WiFiConnectMacro() DWIFIcK::connect(szSsid, key, &status)

#elif defined(USE_WEP40)

const int iWEPKey = 0;
DWIFIcK::WEP40KEY keySet = {    
  0xBE, 0xC9, 0x58, 0x06, 0x97,     // Key 0
  0x00, 0x00, 0x00, 0x00, 0x00,     // Key 1
  0x00, 0x00, 0x00, 0x00, 0x00,     // Key 2
  0x00, 0x00, 0x00, 0x00, 0x00 };   // Key 3
#define WiFiConnectMacro() DWIFIcK::connect(szSsid, keySet, iWEPKey, &status)

#elif defined(USE_WEP104)

const int iWEPKey = 0;
DWIFIcK::WEP104KEY keySet = {   
  0x3E, 0xCD, 0x30, 0xB2, 0x55, 0x2D, 0x3C, 0x50, 0x52, 0x71, 0xE8, 0x83, 0x91,   // Key 0
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   // Key 1
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   // Key 2
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // Key 3
#define WiFiConnectMacro() DWIFIcK::connect(szSsid, keySet, iWEPKey, &status)

#elif defined(USE_WF_CONFIG_H)

#define WiFiConnectMacro() DWIFIcK::connect(0, &status)

#else   // no security - OPEN

#define WiFiConnectMacro() DWIFIcK::connect(szSsid, &status)

#endif

//******************************************************************************************
//******************************************************************************************
//***************************** END OF CONFIGURATION ***************************************
//******************************************************************************************
//******************************************************************************************

typedef enum
{
  NONE = 0,
  WRITE,
  READ,
  CLOSE,
  DONE,
} 
STATE;

STATE state = WRITE;

unsigned tStart = 0;
unsigned tWait = 5000;

// must have a datagram cache
byte rgbDatagramCache[2048];
UdpClient udpClient(rgbDatagramCache, sizeof(rgbDatagramCache));

// our sketch datagram buffer
byte rgbRead[1024];
int cbRead = 0;

// this is for udpClient.writeDatagram to write
byte rgbWriteDatagram[] = { 
  '1','.','3','|',//降水量        两位 一位是小数  0
  ' ','1','6','0','|',//P2.5                    4
  ' ','6','5','|',//湿度                         9
  ' ','2','0','.','8','|',//温度                13
  '2','.','8','|',//风速                        19
  '1','1','1','1','|',//光照                    23
  ' ','9','6','4','.','4','|',//气压            28
  '1','|',//wind direction                     35
  '3','0','.','6','9','9','|',//纬度            37
  '1','0','4','.','0','4','8','\n'};//经度      44
int cbWriteDatagram = sizeof(rgbWriteDatagram);




/*******Water_Level**********/
int analogPin = A11; //水位传感器连接到模拟口1
int val = 0; //定义变量val 初值为0
int data = 0; //定义变量data 初值为0

/********温湿度**********/
#include <dht11.h>
dht11 DHT11;
#define DHT11PIN 10

/*******PM2.5**********/
int dustPin=A0;
float dustVal=0;
int ledPower=2;
int delayTime=280;
int delayTime2=40;
float offTime=9680;

/*******Wind_Speed**********/
float Wind_Speed=0;
int wind_count=0;
int pbIn = 3;

/*******Light_Speed**********/
int Light = A6;
int Light_val=0;

float Water_Level_Update=0;
float PM25_Update=0;
int   Humidity_Update=0;
float Temperature_Update=0;
float Wind_Speed_Update=0;
float Light_Update=0;

/*******Air Pressure**********/
int AirPressure = 0;

/*******Wind_Dir**********/
//int WindSpeed = A0;    // select the input pin for the potentiometer
int WindDir = A1;    // select the input pin for the potentiometer
int sensorValue = 0;  // variable to store the value coming from the sensor
//float WindSpeedValue = 0;
float WindDirValue = 0;


/*******GPS**********/
 //int rxPin = 0;                    // RX PIN 
// int txPin = 1;                    // TX TX
 int ledPin = 13;                  // LED test pin
 int byteGPS=-1;
 char linea[300] = "";
 char comandoGPR[7] = "$GPRMC";
 int cont=0;
 int bien=0;
 int conta=0;
 int indices[13];
 
 
/***      void setup()
 *
 *      Parameters:
 *          None
 *              
 *      Return Values:
 *          None
 *
 *      Description: 
 *      
 *      Arduino setup function.
 *      
 *      Initialize the Serial Monitor, and initializes the
 *      connection to the UDPEchoServer
 *      Use DHCP to get the IP, mask, and gateway
 *      by default we connect to port 44400
 *      
 * ------------------------------------------------------------ */


void SD_Setup()
{
  Serial.print("Initializing SD card...");

  // Make sure the default chip select pin is set to so that
  // shields that have a device that use the default CS pin
  // that are connected to the SPI bus do not hold drive bus
  pinMode(chipSelect_SD_default, OUTPUT);
  digitalWrite(chipSelect_SD_default, HIGH);

  pinMode(chipSelect_SD, OUTPUT);
  digitalWrite(chipSelect_SD, HIGH);

  if (!SD.begin(51)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open("test.txt", FILE_WRITE);

  // if the file opened okay, write to it:
  if (myFile) {
    Serial.print("Writing to test.txt...");
    myFile.println("testing 1, 2, 3.");
    // close the file:
    myFile.close();
    Serial.println("done.");
  } 
  else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }

  // re-open the file for reading:
  myFile = SD.open("test.txt");
  if (myFile) {
    Serial.println("test.txt:");

    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    // close the file:
    myFile.close();
  } 
  else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
}




void WIFI_Setup()
{
  DNETcK::STATUS status;
  int conID = DWIFIcK::INVALID_CONNECTION_ID;


  Serial.println("WiFiUDPEchoClient 1.0");
  Serial.println("Digilent, Copyright 2012");
  Serial.println("");

  if((conID = WiFiConnectMacro()) != DWIFIcK::INVALID_CONNECTION_ID)
  {
    Serial.print("Connection Created, ConID = ");
    Serial.println(conID, DEC);
    state = WRITE;
  }
  else
  {
    Serial.print("Unable to connection, status: ");
    Serial.println(status, DEC);
    state = CLOSE;
  }

  // use DHCP to get our IP and network addresses
  DNETcK::begin();

  // make a connection to our echo server
  udpClient.setEndPoint(szIPServer, portServer);
}

void Sensors_Setup()
{
  pinMode(ledPower,OUTPUT);
  pinMode(dustPin, INPUT);
  Serial.println("DHT11 TEST PROGRAM ");
  Serial.print("LIBRARY VERSION: ");
  Serial.println(DHT11LIB_VERSION);
  Serial.println();

  // MsTimer2::set(1000, Caculate_Wind_Speed);        // 中断设置函数，每 1s 进入一次中断
  //  MsTimer2::start();
  attachInterrupt(pbIn, stateChange, FALLING);
  bmp.begin();  
}

void GPS_Setup()
{
  Serial1.begin(9600);
   for (int i=0;i<300;i++){       // Initialize a buffer for received data
     linea[i]=' ';
   }   
}

void setup() {
  Serial.begin(9600);
  SD_Setup();
  WIFI_Setup();
  GPS_Setup();
  Sensors_Setup();
}

/***      void loop()
 *
 *      Parameters:
 *          None
 *              
 *      Return Values:
 *          None
 *
 *      Description: 
 *      
 *      Arduino loop function.
 *      
 *      We are using the default timeout values for the DNETck and UdpClient class
 *      which usually is enough time for the Udp functions to complete on their first call.
 *
 *      This code will write a sting to the server and have the server echo it back
 *      Remember, UDP is unreliable, so the server may not echo back if the datagram is lost
 *      
 * ------------------------------------------------------------ */
void loop() {
  Light_Level();
  Pressure();
  Caculate_Wind_Speed();
  Updata_Water_Level();
  Updata_Temperature_Humidity();
  Updata_PM25();
  WindDirUpdata();
  GPS_Updata();
  WIFI_Updata();
  //    Serial.print("Temperature = ");
  //    Serial.print(bmp.readTemperature());
  //    Serial.println(" *C");
  // 
  //    Serial.print("Pressure = ");
  //    Serial.print(bmp.readPressure());
  //    Serial.println(" Pa");
  // 
  //    Serial.println();

  delay(500);
}




void WIFI_Updata()
{
  int cbRead = 0;

  switch(state)
  {

    // write out the strings  
  case WRITE:
    if(udpClient.isEndPointResolved())
    {     
      Serial.println("Writing out Datagram");

      udpClient.writeDatagram(rgbWriteDatagram, cbWriteDatagram);
      // udpClient.writeDatagram(12, 2);
      //   udpClient.writeDatagram("|", 2);
      delay(1000);
      //     Serial.println("Waiting to see if a datagram comes back:");
      state = WRITE;         //state = READ;
      tStart = (unsigned) millis();
    }
    break;

    // look for the echo back
  case READ:

    // see if we got anything to read
    if((cbRead = udpClient.available()) > 0)
    {

      cbRead = cbRead < sizeof(rgbRead) ? cbRead : sizeof(rgbRead);
      cbRead = udpClient.readDatagram(rgbRead, cbRead);

      for(int i=0; i < cbRead; i++) 
      {
        Serial.print(rgbRead[i], BYTE);
      }

      // give us some more time to wait for stuff to come back
      tStart = (unsigned) millis();
    }

    // give us some time to get everything echo'ed back
    // or if the datagram is never echoed back
    else if( (((unsigned) millis()) - tStart) > tWait )
    {
      Serial.println("Done waiting, assuming nothing more is coming");
      Serial.println("");
      state = CLOSE;
    }
    break;

    // done, so close up the tcpClient
  case CLOSE:
    udpClient.close();
    Serial.println("Closing udpClient, Done with sketch.");
    state = DONE;
    break;

  case DONE:
  default:
    break;
  }

  // keep the stack alive each pass through the loop()
  DNETcK::periodicTasks(); 
}




/***********************************/
void Light_Level()
{
  Light_val = analogRead(Light); //读取模拟值送给变量val
  Light_Update = Light_val;
  rgbWriteDatagram[23] = (Light_val/1000+48);
  Serial.print("Light:");
  Serial.println(Light_Update);
  rgbWriteDatagram[24] = (Light_val%1000)/100+48;
  rgbWriteDatagram[25] = (Light_val%100)/10+48;
  rgbWriteDatagram[26] = Light_val%10+48;
  // Serial.print("Light_val: "); //串口打印变量data
  // Serial.println(Light_val); //串口打印变量data

}


/***********************************/
void Caculate_Wind_Speed()
{
  int temp=0;
  Wind_Speed = (float)wind_count/2.4;
  Wind_Speed_Update = Wind_Speed;
  temp = (int)Wind_Speed;
  rgbWriteDatagram[19] = temp/10+48;
  rgbWriteDatagram[21] = temp%10+48;
  wind_count=0;
}
void Updata_Wind_Speed()
{
  //Serial.print("Wind Speed: "); //串口打印变量data
  //Serial.println(Wind_Speed,2); //串口打印变量data
}
void stateChange()
{
  wind_count = wind_count+1;
}



/***********************************/
void Updata_Water_Level()
{
  val = analogRead(analogPin); //读取模拟值送给变量val
  Serial.print("Water:");
  Serial.println(val);
  Water_Level_Update = val%1000; //变量val 赋值给变量data
  rgbWriteDatagram[0] = (byte)Water_Level_Update%10+48;
  rgbWriteDatagram[2] = (byte)(Water_Level_Update*10)%10+48;
  // Serial.println("_______________________");
  //  Serial.print("Water_Level:");
  //  Serial.println(data,2); //串口打印变量data
  // delay(100);
}



/***********************************/
void Updata_PM25()
{
  int temp= 0;
  // ledPower is any digital pin on the arduino connected to Pin 3 on the sensor
  digitalWrite(ledPower,LOW); 
  delayMicroseconds(delayTime);
  dustVal=analogRead(dustPin); 
  delayMicroseconds(delayTime2);
  digitalWrite(ledPower,HIGH); 
  delayMicroseconds(offTime);
  // delay(100);
  // Serial.print("PM2.5:  ");
  //  Serial.println((float(dustVal/1024)-0.0356)*120000*0.035,2);
  PM25_Update = (float(dustVal/1024)-0.0356)*120000*0.035;
  Serial.println(PM25_Update);
  temp = (int)PM25_Update/5;
  Serial.println(temp);
  rgbWriteDatagram[5] = temp/100+48;
  rgbWriteDatagram[6] = (temp%100)/10+48;
  rgbWriteDatagram[7] = temp%10+48;
  
}


/***********************************/
void Updata_Temperature_Humidity()
{  
  int temp = 0;
  int Temperature_temp = 0 ;
  int chk = DHT11.read(DHT11PIN);
  //Serial.print("Read sensor: ");
  switch (chk)
  {
  case DHTLIB_OK: 
    //   Serial.println("OK"); 
    break;
  case DHTLIB_ERROR_CHECKSUM: 
    Serial.println("Checksum error"); 
    break;
  case DHTLIB_ERROR_TIMEOUT: 
    Serial.println("Time out error"); 
    break;
  default: 
    Serial.println("Unknown error"); 
    break;
  }

  // Serial.print("Humidity (%): ");
  //  Serial.println((float)DHT11.humidity, 2);
  Humidity_Update = (float)DHT11.humidity;
  temp = Humidity_Update%100;
  if(temp == 0)
    rgbWriteDatagram[9] = 0;
  else
    rgbWriteDatagram[9] = 0;
  rgbWriteDatagram[10] = (Humidity_Update%100)/10+48;
  rgbWriteDatagram[11] = (Humidity_Update%10)+48;
  //  Serial.print("Temperature (oC): ");
  //  Serial.println((float)DHT11.temperature, 2);

  Temperature_Update = (DHT11.temperature) * 10;
  Serial.println(Temperature_Update);
  Temperature_temp = (int)Temperature_Update;
  temp = Temperature_temp/1000;
  if(temp == 0)
    rgbWriteDatagram[13] = 0;
  else
    rgbWriteDatagram[13] = temp+48;
  rgbWriteDatagram[14] = (Temperature_temp%1000)/100+48;
  rgbWriteDatagram[15] = (Temperature_temp%100)/10+48;
  rgbWriteDatagram[17] = (Temperature_temp%10)+48;
  // delay(2000);
}



double Kelvin(double celsius)
{
  return celsius + 273.15;
}     //摄氏温度转化为开氏温度

// 露点（点在此温度时，空气饱和并产生露珠）
// 参考: http://wahiduddin.net/calc/density_algorithms.htm 
double dewPoint(double celsius, double humidity)
{
  double AA0= 373.15/(273.15 + celsius);
  double SUM = -7.90298 * (AA0-1);
  SUM += 5.02808 * log10(AA0);
  SUM += -1.3816e-7 * (pow(10, (11.344*(1-1/AA0)))-1) ;
  SUM += 8.1328e-3 * (pow(10,(-3.49149*(AA0-1)))-1) ;
  SUM += log10(1013.246);
  double VP = pow(10, SUM-3) * humidity;
  double T = log(VP/0.61078);   // temp var
  return (241.88 * T) / (17.558-T);
}

// 快速计算露点，速度是5倍dewPoint()
// 参考: http://en.wikipedia.org/wiki/Dew_point
double dewPointFast(double celsius, double humidity)
{
  double a = 17.271;
  double b = 237.7;
  double temp = (a * celsius) / (b + celsius) + log(humidity/100);
  double Td = (b * temp) / (a - temp);
  return Td;
}

void Pressure()
{      
  int temp = 0;
  AirPressure = bmp.readPressure()/10;
  temp = (AirPressure%100000)/10000;
  if(temp == 0)
    rgbWriteDatagram[28] = 0;
  else
    rgbWriteDatagram[28] = temp+48;
  rgbWriteDatagram[29] = (AirPressure%10000)/1000+48;
  rgbWriteDatagram[30] = (AirPressure%1000)/100+48;
  rgbWriteDatagram[31] = (AirPressure%100)/10+48;
  rgbWriteDatagram[33] = AirPressure%10+48;
}

void WindDirUpdata()
{
  WindDirValue = analogRead(WindDir);    
  Serial.print("V:");
  Serial.println(WindDirValue);
  WindDirValue = ((WindDirValue*5/1023)-0.4)/16*360;
  Serial.print("windDir:");
  Serial.println(WindDirValue);
  if(WindDirValue<4.5)
    rgbWriteDatagram[35] = 1+48;
  if(WindDirValue>=4.5 && WindDirValue <9)
    rgbWriteDatagram[35] = 2+48;
  if(WindDirValue>=9 && WindDirValue <13.5)
    rgbWriteDatagram[35] = 3+48;
  if(WindDirValue>=13.5 && WindDirValue <18)
    rgbWriteDatagram[35] = 4+48;
    
  if(WindDirValue>=18 && WindDirValue <22.5)
    rgbWriteDatagram[35] = 5+48;
  if(WindDirValue>=22.5 && WindDirValue <27)
    rgbWriteDatagram[35] = 6+48;
  if(WindDirValue>=27 && WindDirValue <31.5)
    rgbWriteDatagram[35] = 7+48;
  if(WindDirValue>=31.5)
    rgbWriteDatagram[35] = 8+48;
    
    
  
}

void GPS_Updata()
{
  digitalWrite(ledPin, HIGH);
  byteGPS=Serial1.read();         // Read a byte of the serial port
   if (byteGPS == -1) {           // See if the port is empty yet
     delay(100); 
   } else {
     // note: there is a potential buffer overflow here!
     linea[conta]=byteGPS;        // If there is serial port data, it is put in the buffer
     conta++;                      
     Serial.print(byteGPS, BYTE); 
     if (byteGPS==13){            // If the received byte is = to 13, end of transmission
       // note: the actual end of transmission is <CR><LF> (i.e. 0x13 0x10)
       digitalWrite(ledPin, LOW); 
       cont=0;
       bien=0;
       // The following for loop starts at 1, because this code is clowny and the first byte is the <LF> (0x10) from the previous transmission.
       for (int i=1;i<7;i++){     // Verifies if the received command starts with $GPR
         if (linea[i]==comandoGPR[i-1]){
           bien++;
         }
       }
       if(bien==6){               // If yes, continue and process the data
         for (int i=0;i<300;i++){
           if (linea[i]==','){    // check for the position of the  "," separator
             // note: again, there is a potential buffer overflow here!
             indices[cont]=i;
             cont++;
           }
           if (linea[i]=='*'){    // ... and the "*"
             indices[12]=i;
             cont++;
           }
         }
         Serial.println("");      // ... and write to the serial port
         Serial.println("");
         Serial.println("---------------");
         for (int i=0;i<12;i++){
           switch(i){
             case 0 :Serial.print("Time in UTC (HhMmSs): ");break;
             case 1 :Serial.print("Status (A=OK,V=KO): ");break;
             case 2 :Serial.print("Latitude: ");break;
             case 3 :Serial.print("Direction (N/S): ");break;
             case 4 :Serial.print("Longitude: ");break;
             case 5 :Serial.print("Direction (E/W): ");break;
             case 6 :Serial.print("Velocity in knots: ");break;
             case 7 :Serial.print("Heading in degrees: ");break;
             case 8 :Serial.print("Date UTC (DdMmAa): ");break;
             case 9 :Serial.print("Magnetic degrees: ");break;
             case 10 :Serial.print("(E/W): ");break;
             case 11 :Serial.print("Mode: ");break;
             case 12 :Serial.print("Checksum: ");break;
           }
           for (int j=indices[i];j<(indices[i+1]-1);j++){
             Serial.print(linea[j+1]); 
           }
           Serial.println("");
         }
         Serial.println("---------------");
       }
       conta=0;                    // Reset the buffer
       for (int i=0;i<300;i++){    //  
         linea[i]=' ';             
       }                 
     }
   }
}

