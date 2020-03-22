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
#include <Fonts/FreeMonoBold18pt7b.h>
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
const char* ssid = "freebox_HWWVKE";
const char* password = "virginie75";

//Your IP address or domain name with URL path

const char* serverNameTarg = "http://192.168.0.35/target";
const char* serverNameTws = "http://192.168.0.35/tws";
const char* serverNameSog = "http://192.168.0.35/sog";
const char* serverNameTwa = "http://192.168.0.35/twa";

String tws;
String twa;
String target;
String sog;
int perf;

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
      sog = httpGETRequest(serverNameSog);
      target = httpGETRequest(serverNameTarg);
      tws = httpGETRequest(serverNameTws);
      twa = httpGETRequest(serverNameTwa);
      Serial.println("sog " + sog +  " perf" + perf + " tgt" + target + " tws" + tws );

      tft.fillScreen(WHITE);

      // display cog
      tft.setFont();
      tft.setTextSize(1);
      tft.setTextColor(BLACK);
      tft.setCursor(0, 0);
      tft.print("sog:");
      int s = sog.toInt();
      tft.print(float(s) / 10, 1);


      // display tws

      tft.setCursor(48, 0);
      tft.print("twa");
      tft.print(twa);

      tft.setCursor(88, 0);
      tft.print("tws");
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
        tft.setFont(&FreeMonoBold18pt7b);
        if (perf < 100) {
          tft.print(" ");
        }
        tft.print(perf);
      }

      //tft.print(target);
      //display.display();

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
