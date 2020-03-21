/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-client-server-wi-fi/

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <WiFi.h>
#include <HTTPClient.h>
#include <SSD1283A.h> //Hardware-specific library
#include <Fonts/FreeMonoBoldOblique12pt7b.h>
#include <Fonts/FreeSerif9pt7b.h>

#include <Fonts/FreeMonoBold24pt7b.h>
SSD1283A tft(/*CS=5*/ SS, /*A0DC=*/ 17, /*RST=*/ 16, /*LED=*/ 4); //hardware spi,cs,cd,reset,led SCL=18 SDA=23MOSI

#define  BLACK  0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

// Set your network
const char* ssid = "yourssid";
const char* password = "yourpassword";

//Your IP address or domain name with URL path
const char* serverNameTemp = "http://192.168.0.35/temperature";
const char* serverNamePres = "http://192.168.0.35/pressure";
const char* serverNametarg = "http://192.168.0.35/target";
const char* serverNametwa = "http://192.168.0.35/twa";
const char* serverNametws = "http://192.168.0.35/tws";

String temperature;
String pressure;
String target;
String twa;
String tws;
float flag = 0;
float i = 0;
unsigned long previousMillis = 0;
const long interval = 1000;

void setup() {
  Serial.begin(115200);
  tft.init();
  tft.fillScreen(WHITE);

  // display temperature
  tft.setTextSize(1);
  tft.setTextColor(BLACK);
  tft.setCursor(0, 0);
  tft.println("tft.init() done");
  Serial.println("tft.init() done");


  WiFi.begin(ssid, password);
  tft.println("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  tft.println("Connected to WiFi network with IP Address: ");
  tft.print(WiFi.localIP());
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());
  delay(500);
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // Check WiFi connection status
    if (WiFi.status() == WL_CONNECTED ) {
      temperature = httpGETRequest(serverNameTemp);
      pressure = httpGETRequest(serverNamePres);
      target = httpGETRequest(serverNametarg);
      twa = httpGETRequest(serverNametwa);
      tws = httpGETRequest(serverNametws);
if (i>22){
flag = 0;
}
 if (i<0){
flag = 0;
}
 if (flag==0){
i=i+0,1;
}  
 
 if (flag==1){
i=i+0,1;
}    
      Serial.println("Temperature: " + temperature + " Pressure: " + pressure + " hPa");

      tft.fillScreen(WHITE);

      // display temperature
      tft.setFont();
      tft.setTextSize(2);
      tft.setTextColor(BLACK);
      tft.setCursor(0, 0);
      tft.print("T:");
      tft.print(temperature);
      tft.print(" ");
      tft.setTextSize(2);
      //display.cp437(true);
      tft.write(248);
      tft.setTextColor(RED);
      tft.setTextSize(2);
      tft.print("C");

      // display pressure
      tft.setTextSize(2);
      tft.setCursor(0, 16);
      tft.print("P:");
      tft.print(pressure);
      tft.setTextSize(1);
      tft.setCursor(110, 22);
      tft.print("hPa");




      // display twa
      tft.setTextSize(2);
      tft.setCursor(0, 32);
      tft.print("Twa:");
      tft.print(twa);


      // display tws
      tft.setTextSize(2);
      tft.setCursor(0, 45);
      tft.print("Tws:");
      tft.print(tws);

      // display target
      tft.setTextColor(GREEN);
      tft.setTextSize(1);
      tft.setCursor(0, 66);
      tft.print("Tgt");
      tft.setCursor(20, 102);
      tft.setTextSize(1);
      //tft.print(target);
      tft.setFont(&FreeMonoBold24pt7b);
      tft.print(i,1);
      //tft.print(target);
      //display.display();

      // save the last HTTP GET Request
      previousMillis = currentMillis;
    }
    else {
      Serial.println("WiFi Disconnected");
    }
  }
}

String httpGETRequest(const char* serverName) {
  HTTPClient http;

  // Your IP address with path or Domain name with URL path
  http.begin(serverName);

  // Send HTTP POST request
  int httpResponseCode = http.GET();

  String payload = "--";

  if (httpResponseCode > 0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    payload = http.getString();
  }
  else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }
  // Free resources
  http.end();

  return payload;
}
