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

/************************************************************************/
/*                                                                      */
/*              Include ONLY 1 hardware library that matches            */
/*              the network hardware you are using                      */
/*                                                                      */
/*              Refer to the hardware library header file               */
/*              for supported boards and hardware configurations        */
/*                                                                      */
/************************************************************************/
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
/************************************************************************/
/*                                                                      */
/*              SET THESE VALUES FOR YOUR NETWORK                       */
/*                                                                      */
/************************************************************************/

char * szIPServer = "192.168.165.2";
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

    DWIFIcK::WPA2KEY key = { 0x27, 0x2C, 0x89, 0xCC, 0xE9, 0x56, 0x31, 0x1E, 
                            0x3B, 0xAD, 0x79, 0xF7, 0x1D, 0xC4, 0xB9, 0x05, 
                            0x7A, 0x34, 0x4C, 0x3E, 0xB5, 0xFA, 0x38, 0xC2, 
                            0x0F, 0x0A, 0xB0, 0x90, 0xDC, 0x62, 0xAD, 0x58 };
    #define WiFiConnectMacro() DWIFIcK::connect(szSsid, key, &status)

#elif defined(USE_WEP40)

    const int iWEPKey = 0;
    DWIFIcK::WEP40KEY keySet = {    0xBE, 0xC9, 0x58, 0x06, 0x97,     // Key 0
                                    0x00, 0x00, 0x00, 0x00, 0x00,     // Key 1
                                    0x00, 0x00, 0x00, 0x00, 0x00,     // Key 2
                                    0x00, 0x00, 0x00, 0x00, 0x00 };   // Key 3
    #define WiFiConnectMacro() DWIFIcK::connect(szSsid, keySet, iWEPKey, &status)

#elif defined(USE_WEP104)

    const int iWEPKey = 0;
    DWIFIcK::WEP104KEY keySet = {   0x3E, 0xCD, 0x30, 0xB2, 0x55, 0x2D, 0x3C, 0x50, 0x52, 0x71, 0xE8, 0x83, 0x91,   // Key 0
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
} STATE;

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
byte rgbWriteDatagram[] = { '1','.','3','|',//降水量        两位 一位是小数  0
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
#define DHT11PIN 3

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
float Humidity_Update=0;
float Temperature_Update=0;
float Wind_Speed_Update=0;
float Light_Update=0;

/*******Air Pressure**********/
int AirPressure = 0;


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




void setup() {
    Serial.begin(9600);
    WIFI_Setup();
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
  Serial.println(rgbWriteDatagram[23]);
  rgbWriteDatagram[24] = (Light_val%1000)/100+48;
  rgbWriteDatagram[25] = (Light_val%100)/10+48;
  rgbWriteDatagram[26] = Light_val%10+48;
 // Serial.print("Light_val: "); //串口打印变量data
 // Serial.println(Light_val); //串口打印变量data
  
}


/***********************************/
void Caculate_Wind_Speed()
{
   Wind_Speed = (float)wind_count/24;
   Wind_Speed_Update = Wind_Speed;
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
    Water_Level_Update = val/1000; //变量val 赋值给变量data
    rgbWriteDatagram[0] = (byte)Water_Level_Update%10;
    rgbWriteDatagram[2] = (byte)(Water_Level_Update*10)%10;
   // Serial.println("_______________________");
  //  Serial.print("Water_Level:");
  //  Serial.println(data,2); //串口打印变量data
   // delay(100);
}



/***********************************/
void Updata_PM25()
{
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
}


/***********************************/
void Updata_Temperature_Humidity()
{
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
    //  Serial.print("Temperature (oC): ");
    //  Serial.println((float)DHT11.temperature, 2);
      Temperature_Update = (float)DHT11.temperature;
     // delay(2000);
}


double Fahrenheit(double celsius) 
{
        return 1.8 * celsius + 32;
}    //摄氏温度度转化为华氏温度

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

