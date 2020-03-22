/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-client-server-wi-fi/

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/
#include <LiquidCrystal_PCF8574.h>
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
/*#include <SSD1283A.h> //Hardware-specific library
  //#include <Fonts/FreeMonoBoldOblique12pt7b.h>
  //#include <Fonts/FreeMonoBold18pt7b.h>
  //#include <Fonts/FreeMonoBold24pt7b.h>

  #define  BLACK  0x0000
  #define BLUE    0x001F
  #define RED     0xF800
  #define GREEN   0x07E0
  #define CYAN    0x07FF
  #define MAGENTA 0xF81F
  #define YELLOW  0xFFE0
  #define WHITE   0xFFFF
*/

//SSD1283A tft(/*CS=5*/ SS, /*A0DC=*/ 17, /*RST=*/ 16, /*LED=*/ 4); //hardware spi,cs,cd,reset,led SCL=18 SDA=23MOSI
//lcd
#define SDA_PIN 4
#define SCL_PIN 15
LiquidCrystal_PCF8574 lcd(0x27); // set the LCD address to 0x27 for a 16 chars and 2 line display

int show = -1;
// Set your network
const char* ssid = "yourssid";
const char* password = "yourpassword";

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
  int error;
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.beginTransmission(0x27);
  error = Wire.endTransmission();

  if (error == 0) {
    Serial.println(": LCD found.");
    show = 0;
    lcd.begin(20, 4); // initialize the lcd

  } else {
    Serial.println(": LCD not found.");
  } // if

  /*
    tft.init();
    tft.fillScreen(WHITE);
    tft.setRotation(1);
    // display temperature
    tft.setTextSize(1);
    tft.setTextColor(BLACK);
    tft.setCursor(0, 0);
    tft.println("tft.init() done");
    Serial.println("tft.init() done");

  */
  WiFi.begin(ssid, password);
  lcd.println("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    lcd.print(".");
  }
  Serial.println("");
  lcd.setBacklight(100);
  lcd.home();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Connected to WiFi");
  lcd.setCursor(0, 1);
  lcd.print("IP Address: ");
  lcd.setCursor(0, 2);
  lcd.print(WiFi.localIP());
  delay(4000);

  /*
    tft.println("Connected to WiFi network with IP Address: ");
    tft.print(WiFi.localIP());
    Serial.print("Connected to WiFi network with IP Address: ");
    Serial.println(WiFi.localIP());
    delay(500);*/
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
      /*TFT
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

        tft.setCursor(60, 0);
        tft.print("twa:");
        tft.print(twa);
        tft.setCursor(60, 12);
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
      */
      //LCD
      // display cog
      lcd.home();
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("sog ");
      int s = sog.toInt();
      lcd.print(float(s) / 10, 1);

      // display twa tws

      lcd.setCursor(0, 1);
      lcd.print("twa ");
      lcd.print(twa);
      lcd.setCursor(10, 1);
      lcd.print("tws ");
      lcd.print(tws);

      // display Perf

      lcd.setCursor(0, 2);
      lcd.print("perf ");
      int t = target.toInt();
      if (t != 0) {
        perf = (int(float(s) / float(t) * 100));

        if (perf < 100) {
          lcd.print(" ");
        }
        lcd.print(perf);
        lcd.print(" %");
      }
      // display target

      lcd.setCursor(0, 3);
      lcd.print("Tgt:");
      if (target.toInt() < 100) {
        lcd.print(" ");
      }
      lcd.print(float(target.toInt()) / 10, 1);



      // save the last HTTP GET Request
      previousMillis = currentMillis;
    }
    else {
      /*// TFT

        tft.fillScreen(WHITE);
        Serial.println("WiFi Disconnected");
        tft.setFont();
        tft.setTextSize(2);
        tft.setTextColor(BLACK);
        tft.setCursor(0, 0);
        tft.print("WiFi Disconnected"); */
      //LCD
      Serial.println("WiFi Disconnected");
      lcd.setCursor(0, 0);
      lcd.print("WiFi Disconnected");

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
