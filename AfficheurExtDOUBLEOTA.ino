/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-client-server-wi-fi/

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <HTTPClient.h>
#include <SSD1283A.h> //Hardware-specific library
#include <Fonts/FreeMonoBoldOblique12pt7b.h>
#include <Fonts/FreeMonoBold18pt7b.h>
#include <Fonts/FreeMonoBold24pt7b.h>
#include <Average.h>

SSD1283A tft(/*CS=5*/ 33, /*A0DC=*/ 17, /*RST=*/ 32, /*LED=*/ 4); //hardware spi,cs,cd,reset,led SCL=18 SDA=23MOSI
SSD1283A tft2(/*CS=5*/ 5, /*A0DC=*/ 17, /*RST=*/ 16, /*LED=*/ 4); //hardware spi,cs,cd,reset,led SCL=18 SDA=23MOSI

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
const char* password = "yourpass";

//Your IP address or domain name with URL path
AsyncWebServer server(80);
const char* serverNameTarg = "http://192.168.50.36/target";
const char* serverNameTws = "http://192.168.50.36/tws";
const char* serverNameSog = "http://192.168.50.36/sog";
const char* serverNameTwa = "http://192.168.50.36/twa";

String tws;
String twa;
String target;
String sog;
int perf;
Average<float> aveperf(10);
int perfa;
unsigned long previousMillis = 0;
const long interval = 1000;

void setup() {
  Serial.begin(115200);
  tft.init();
  tft.fillScreen(WHITE);
  tft.setRotation(2);
  tft.setTextSize(1);
  tft.setTextColor(BLACK);
  tft.setCursor(0, 0);
  tft.println("tft.init() done");
  Serial.println("tft.init() done");

  tft2.init();
  tft2.fillScreen(WHITE);
  tft2.setRotation(2);
  tft2.setTextSize(1);
  tft2.setTextColor(BLACK);
  tft2.setCursor(0, 0);
  tft2.println("tft2.init() done");
  Serial.println("tft2.init() done");



  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(200, "text/plain", "Afficheurdouble  /update pour maj");
  });

  AsyncElegantOTA.begin(&server);
  Serial.println("");
  tft.println("Connected to WiFi network with IP Address: ");
  tft.print(WiFi.localIP());
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());
  delay(500);
  server.begin();

}

void loop() {
  unsigned long currentMillis = millis();
  AsyncElegantOTA.loop();
  if (currentMillis - previousMillis >= interval) {
    // Check WiFi connection status
    if (WiFi.status() == WL_CONNECTED ) {
      sog = httpGETRequest(serverNameSog);
      target = httpGETRequest(serverNameTarg);
      tws = httpGETRequest(serverNameTws);
      twa = httpGETRequest(serverNameTwa);
      Serial.println("sog " + sog +  " perf" + perf + " tgt" + target + " tws" + tws );
      tft2.fillScreen(WHITE);
      tft.fillScreen(WHITE);

      // display Sog
      tft2.setFont();
      tft2.setTextSize(2);
      tft2.setTextColor(BLACK);
      tft2.setCursor(0, 0);
      tft2.print("sog:");
      int s = sog.toInt();
      tft2.print(float(s) / 10, 1);


      // display tws TWA
      tft.setFont();
      tft.setTextSize(2);
      tft.setCursor(0, 0);
      tft.print("twa:");
      tft.print(twa);
      tft.setCursor(0, 12);
      tft.print("tws:");
      tft.print(tws);
     
      
      // display Perf
      tft.setTextColor(BLUE);
      tft.setTextSize(1);
      tft.setCursor(0, 24);
      tft.print("perf");

      tft.setCursor(0, 70);
      tft.setTextSize(2);
      int t = target.toInt();
      if (t != 0) {
        perf = (int(float(s) / float(t) * 100));
        aveperf.push(perf);
        perfa = (aveperf.mean(), 0);
        tft.setFont(&FreeMonoBold18pt7b);
        if (perfa < 100) {
          tft.print(" ");
        }
        tft.print(perfa);
      }

      //tft.print(target);
      //display.display();
      // display target ECRAN 2
      tft2.setTextColor(BLACK);
      tft2.setTextSize(1);
      tft2.setFont();
      tft2.setCursor(0, 74);
      tft2.print("Perf");
      tft2.setCursor(0, 118);
      tft2.setTextSize(2);
      tft2.setFont(&FreeMonoBold18pt7b);
      if (target.toInt() < 100) {
        tft2.print(" ");
      }
      //tft2.print(target);
      tft2.print(perf);
      tft2.setCursor(64, 126);
      tft2.print(".");

      // display target
      tft.setTextColor(BLACK);
      tft.setTextSize(1);
      tft.setFont();
      tft.setCursor(0, 74);
      tft.print("Tgt");
      tft.setCursor(0, 118);
      tft.setTextSize(2);
      tft.setFont(&FreeMonoBold18pt7b);
      if (target.toInt() < 100) {
        tft.print(" ");
      }
      tft.print(target);
      tft.setCursor(64, 126);
      tft.print(".");

      // save the last HTTP GET Request
      previousMillis = currentMillis;
    }
    else {
      tft.fillScreen(WHITE);
      Serial.println("WiFi Disconnected");
      tft.setFont();
      tft.setTextSize(2);
      tft.setTextColor(BLACK);
      tft.setCursor(0, 0);
      tft.print("WiFi Disconnected");
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
