// pour l'utilisation Du NMEA2000 https://github.com/ttlappalainen/NMEA2000
// pour l'interpolation d'une table  https://forum.arduino.cc/index.php?topic=8484.15
// pour le serveur client esp32 https://randomnerdtutorials.com/esp32-client-server-wi-fi/
// pour le calcul de la vitesse cible https://github.com/cape-io/true-wind/blob/master/true-wind.coffee
/*BMP280 Barometric/Temp sensor with OLED display
   I2C ESP32 4 15 SDA 4 SCL 15 SAME AS OLED
   CHANGE ADRESS to 76 IN <Adafruit_BMP280.h> IF NECESSARY
   Wiring Diagram:
   SDA from BMP and OLED to PIN 4
   SCL from BMP and OLED to PIN 15
   GND to GND
   VCC from BMP and OLED to 3V
*/
//ESP32 CAN
#define ESP32_CAN_TX_PIN GPIO_NUM_25 //ORANGE
#define ESP32_CAN_RX_PIN GPIO_NUM_26 //BLANC
//#define I2C_SDA 33  //DEFAULT 21
//#define I2C_SCL 32 // DEFAULT 22

#include <Arduino.h>
#include <NMEA2000_CAN.h>  // This will automatically choose right CAN library and create suitable NMEA2000 object
#include <N2kMsg.h>
#include <NMEA2000.h>
#include <N2kMessages.h>
#include <HTTPClient.h> //POUR RECUPDONNEES
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include "WiFi.h"
#include <Wire.h>
#include <SPI.h>
//#include <Adafruit_Sensor.h>  //POUR BMP
//#include <Adafruit_BMP280.h> //POUR BMP
//#include <Adafruit_SSD1306.h> //POUR OLED
#include <Average.h>
#include <Flash.h>


// Meter colour schemes
#define RED2RED 0
#define GREEN2GREEN 1
#define BLUE2BLUE 2
#define BLUE2RED 3
#define GREEN2RED 4
#define RED2GREEN 5

#define TFT_GREY 0x2104 // Dark grey 16 bit colour

#include "alert.h" // Out of range alert icon
#include <TFT_eSPI.h> // Hardware-specific library
TFT_eSPI tft = TFT_eSPI();  // Invoke library

// VARIABLE RINGMETER
uint32_t runTime = -99999;       // time for next update
int xpos = 0, ypos = 5, gap = 4, radius = 52;
int reading = 0; // Value to be displayed

boolean range_error = 0;
int8_t ramp = 1;


//Wire.begin(I2C_SDA, I2C_SCL);
/*//OLED pins
  #define OLED_SDA 4
  #define OLED_SCL 15
  #define OLED_RST 16
  #define SCREEN_WIDTH 128 // OLED display width, in pixels
  #define SCREEN_HEIGHT 64 // OLED display height, in pixels
*/
#define TFT_GREY 0x5AEB // New colour

// Set your network
const char* SSID = "yourssid";
const char* PASSWORD = "yourpass";

// POUR HTTP CLIENT
const char* serverNameReste = "http://192.168.50.35/Reste";
const char* serverNameDebit = "http://192.168.50.35/Debit";
String Debit;
String Reste;

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

//Adafruit_BMP280 bmp; // use I2C interface
//Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
//Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();
//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

//POLAIRE
const int itwsSteps = 18; // Number of entries for itws
const long itwsFactor = 10; // Difference between entries for itws
const int itwsMin = 0; // Value of the first entry for itws
const int itwsMax = itwsMin + (itwsSteps - 1) * itwsFactor; // Value of the last entry for itws
const int itwaSteps = 24; // Number of entries for itwa
const long itwaFactor = 5; //  Difference between entries for itwa
const int itwaMin = 0; //  Value of the first entry for itwa
const int itwaMax = itwaMin + (itwaSteps - 1) * itwaFactor; // Value of the last entry for itwa
double itws ;
double itwa ;

//10x 0   1   2  3    4   5   6   7   8   9    10   11    12   13   14   15   16   17
//    0,  4,  6, 8,  10, 12, 14, 16, 18,  20,  25,  30,   35,  40,  45,  50,  55,  60
FLASH_TABLE(int, Target, itwsSteps ,
{0,  0,  0,  0,  0,  0,  0,   0,   0,   0,   0,   0,   0,   0,  0,  0,  0,  0},      // 0   0
{0,  1,  2,  2,  3,  4,  4,   5,   5,   5,   6,   5,   5,   0,  0,  0,  0,  0},      // 5   5
{0,  2,  3,  5,  6,  7,  8,   9,   9,  10,  11,  10,  10,   0,  0,  0,  0,  0},      // 10  10
{0,  1,  4,  5,  7, 10, 11,  12,  14,  14,  15,  16,  16,  15,  0,  0,  0,  0},      // 15  15
{0,  4,  6,  8, 11, 12, 14,  16,  17,  18,  18,  18,  18,   0,  0,  0,  0,  0},      // 20  20
{0,  5,  7, 10, 13, 15, 17,  19,  20,  21,  22,  22,  21,   0,  0,  0,  0,  0},      // 25  25
{0,  8, 12, 16, 22, 25, 28,  31,  33,  35,  37,  36,  35,  26, 10,  4,  0,  0},      // 32  30
{0, 16, 24, 32, 42, 48, 53,  57,  61,  64,  68,  66,  62,  51, 20,  6,  0,  0},      // 36  35
{0, 19, 29, 39, 48, 55, 59,  64,  68,  72,  77,  74,  71,  63, 25,  11, 0,  0},      // 40  40
{0, 22, 33, 44, 53, 61, 65,  70,  74,  78,  83,  80,  77,  73, 27,  12, 0,  0},      // 45  45
{0, 25, 37, 50, 59, 68, 73,  78,  77,  87,  92,  88,  85,  83, 30,  13, 0,  0},      // 52  50
{0, 28, 42, 56, 66, 76, 81,  86,  90,  95, 101,  97,  93,  93, 35,  16, 0,  0},      // 60  55
{0, 30, 45, 60, 71, 81, 87,  92,  96, 103, 109, 105, 101, 101, 40,  20, 0,  0},      // 70  60
{0, 32, 48, 64, 77, 86, 92,  98, 104, 110, 117, 113, 109, 109, 46,  25, 0,  0},      // 80  65
{0, 32, 48, 64, 77, 87, 94, 101, 103, 115, 122, 118, 114, 114, 51,  28, 3,  0},      // 90  70
{0, 32, 48, 64, 77, 88, 96, 104, 112, 120, 128, 123, 118, 118, 59,  35, 6,  0},      // 100 75
{0, 30, 45, 60, 74, 84, 93, 101, 105, 119, 127, 122, 117, 117, 64,  41, 9,  3},      // 110 80
{0, 28, 42, 56, 68, 80, 89,  99, 109, 118, 125, 121, 117, 117, 70,  47, 12, 6},      // 120 85
{0, 26, 39, 52, 62, 73, 82,  91, 100, 109, 116, 111, 106, 106, 69,  48, 16, 11},     // 130 90
{0, 24, 36, 48, 58, 67, 75,  83,  91, 100, 106, 102,  98,  98, 74,  54, 20, 15},     // 140 95
{0, 23, 35, 46, 55, 63, 68,  74,  81,  87,  93,  89,  85,  85, 68,  51, 19, 15},     // 150 100
{0, 22, 33, 44, 51, 58, 62,  65,  69,  74,  79,  76,  73,  73, 62,  47, 18, 15},     // 160 105
{0, 20, 30, 40, 46, 53, 56,  59,  63,  65,  69,  67,  65,  65, 62,  49, 20, 16},     // 170 110
{0, 16, 24, 32, 39, 46, 50,  54,  56,  58,  62,  59,  56,  56, 56,  45, 17, 14});    // 180 115

// ------------------FIN POLAIRE-------------------------
//  -----linear---
int value[6] = {0, 0, 0, 0, 0, 0};
int old_value[6] = { -1, -1, -1, -1, -1, -1};
int d = 0;
//

int i;
double awa;
double aws;
double cog;
double sog;
double head;
double twa;
double twd;
double tws;
double target;
Average<float> avetarget(5); //le chiffre entre parenthèse est le nombre de valeurs utilisées
String targ;

//etat nmea2k
const int nmeaPin = 36;
int nmeaState = 0;
int laststate = 0;
// TIMERS
unsigned long previousMillis = 0;
unsigned long previousMilliswifi = 0;
const long intervalwifi = 10000;
const long interval = 2000; // INTERVALLE RAFRAICHISSEMENT



String readsog() {
  return String((sog * 10), 0);
}
String readtarget() {
  return String(avetarget.mean(), 0);
}
String readtws() {
  return String((tws), 1);
}
String readtwa() {
  return String((RadToDeg(twa)), 0);
}
/*
  void startOLED() {
  //reset OLED display via software
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(20);
  digitalWrite(OLED_RST, HIGH);

  //initialize OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }
  //MESS
  }
*/
// List here messages your device will transmit.
typedef struct {
  unsigned long PGN;
  void (*Handler)(const tN2kMsg &N2kMsg);
} tNMEA2000Handler;

//void FluidLevel(const tN2kMsg &N2kMsg);
//void WaterDepth(const tN2kMsg &N2kMsg);
void WindSpeed(const tN2kMsg &N2kMsg);
void COGSOGRapid(const tN2kMsg &N2kMsg);
void Heading(const tN2kMsg &N2kMsg);

tNMEA2000Handler NMEA2000Handlers[] = {
  {130306L, &WindSpeed},
  {129026L, &COGSOGRapid},
  {127250L, &Heading},
  {0, 0}
};

void initWiFi() {
  // Connect to Wi-Fi network with SSID and PASSWORD
  tft.print("Connecting to ");
  tft.println(SSID);
  WiFi.begin(SSID, PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    tft.print(".");
  }
  // Print local IP address
  tft.println("");
  tft.println("WiFi connected at IP address:");
  tft.println(WiFi.localIP());

}

void setup() {
  tft.init();
  tft.setRotation(1);
  Serial.begin(115200);

  // initialize the nmea pin as an input
  pinMode(nmeaPin, INPUT);
  tft.fillScreen(TFT_BLACK);

  int xpos = 0, ypos = 5, gap = 4, radius = 52;

  /*
    if (!bmp.begin()) {
      Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));

      tft.print("BMP SENSOR");
      tft.setCursor(92, 0);
      tft.print("NOT FOUND!");
      delay(4000);
      //while (1);
    }

  */
  // Do not forward bus messages at all
  //NMEA2000.EnableForward(false);
  NMEA2000.SetMsgHandler(HandleNMEA2000Msg);
  NMEA2000.Open();

  initWiFi();

  //delay(5000);

  // pour acceder aux données depuis un client
  server.on("/sog", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", readsog().c_str());
  });
  server.on("/target", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", readtarget().c_str());
  });
  server.on("/tws", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", readtws().c_str());
  });
  server.on("/twa", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", readtwa().c_str());
  });
  AsyncElegantOTA.begin(&server);    // Start ElegantOTA
  server.begin();
  tft.fillScreen(TFT_BLACK);
}

void WindSpeed(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  double WindSpeed;
  double WindAngle;
  tN2kWindReference WindReference;

  if (ParseN2kWindSpeed(N2kMsg, SID, WindSpeed, WindAngle, WindReference) ) {
    Serial.print("Aws");
    Serial.println(WindSpeed);
    Serial.print("AwA");
    Serial.println(WindAngle);
    aws = msToKnots(WindSpeed);
    awa = WindAngle;
    if (aws < 0.1)  {
      aws = 0;
    }
    if (awa < 0.1)  {
      awa = 0;
    }
  }
}
void COGSOGRapid(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  tN2kHeadingReference ref;
  double COG;
  double SOG;

  if (ParseN2kCOGSOGRapid(N2kMsg, SID, ref, COG, SOG) ) {
    Serial.print("Cog");
    Serial.println(COG);
    Serial.print("Sog");
    Serial.println(SOG);
    cog = COG;
    sog = msToKnots(SOG);
  }
}
void Heading(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  tN2kHeadingReference HeadingReference;
  double Heading;
  double Deviation;
  double Variation;

  if (ParseN2kHeading(N2kMsg, SID, Heading, Deviation, Variation, HeadingReference) ) {
    Serial.print("Heading");
    Serial.println(Heading);
    head = Heading;
  }
}

//NMEA 2000 message handler
void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) {
  int iHandler;
  // Find handler
  for (iHandler = 0; NMEA2000Handlers[iHandler].PGN != 0 && !(N2kMsg.PGN == NMEA2000Handlers[iHandler].PGN); iHandler++);
  if (NMEA2000Handlers[iHandler].PGN != 0) {
    NMEA2000Handlers[iHandler].Handler(N2kMsg);
  }
}

void loop() {
  AsyncElegantOTA.loop();
  /* // TOUCH TEST_______________
    uint16_t x, y;

    tft.getTouchRaw(&x, &y);
    tft.setCursor(122, 0);
    tft.print((char)248);
    tft.setCursor(92, 0);

    Serial.printf("X: %i     ", x);
    Serial.printf("y: %i     ", y);

    Serial.printf("z: %i \n", tft.getTouchRawZ());

    // _____TOUCH TEST
  */



  NMEA2000.ParseMessages();
  unsigned long currentMillis = millis();

  if ((WiFi.status() != WL_CONNECTED) && (currentMillis - previousMilliswifi >= intervalwifi)) {
    Serial.print(millis());
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();
    previousMilliswifi = currentMillis;
  }
  if (currentMillis - previousMillis >= interval) {

    previousMillis = currentMillis;

// affichage cuve--------------------------------
    Reste = httpGETRequest(serverNameReste);
    Debit = httpGETRequest(serverNameDebit);


    xpos = 180 + 40, ypos = 0, gap = 15, radius = 130;

    
    reading = Reste.toInt();
    ringMeter(reading, 0, 570, xpos, ypos, radius, "Litres", GREEN2RED); // Draw analogue meter
    // affichage débit
    if ( Debit.toFloat() > 0.01) {
      tft.drawString(Debit, 350,  290, 6);
      tft.drawString("Debit", 350,  250, 4);
    }
    else {
      tft.drawString("   --   ", 350,  290, 6);
      tft.drawString("Debit", 350,  250, 4);
    }
    /*  double temp = bmp.readTemperature();
      Serial.print(F("Temperature = "));
      Serial.print(temp, 1); //     ,# sets the number of decimal places
      Serial.println(" *C");

    */

    //affichage nmea si connecté
    // read the state of the nmea value
    nmeaState = digitalRead(nmeaPin);
    //Serial.println(nmeaState);
    // check if the nmea is on.
    if ((nmeaState == HIGH ) && (laststate == LOW)){
     tft.fillScreen(TFT_BLACK);
    laststate = nmeaState;
    }
    if (nmeaState == HIGH) {
      //LIGNE8 90
      tft.setCursor(0, 9);
      tft.print("nmea off ");
     //tft.fillRect(0, 18, 160, 260, TFT_BLACK);

    }  
    else {
      //LIGNE8 90
      tft.setCursor(0, 9);
      tft.print("nmea on ");
      NmeaOn(); //affichage calcul performance
     laststate = nmeaState;
    }


    //LIGNE1

    tft.setCursor(0, 0);
    tft.print(WiFi.localIP());


    /*

        //LIGNE 8 72

        tft.setCursor(0, 72);
        tft.print( F("Temp: "));
        tft.setCursor(62, 72);
        tft.print(temp, 1); //    ,# sets the number of decimal places
        tft.print(F("   C"));
        tft.setCursor(92, 72);
        tft.print((char)247); // degree symbol

      double pressure = bmp.readPressure() / 100 ;
        tft.setCursor(0, 90);
        tft.print( F("Press: "));
        tft.setCursor(62, 90);
        tft.print(pressure, 1); //   ,# sets the number of decimal places
        tft.print(F(" mB"));*/

  }
}



void NmeaOn() {

  //LIGNE3

  tft.setCursor(0, 18);
  tft.print("TgtS ");//
  if (aws == 0 && awa == 0) {
    target = 0;
  }
  else {
    get_true(sog, cog, aws, head, awa, twd, tws, twa);
    target = (FindTarget((tws * 10), RadToDeg(twa)));
    avetarget.push(target);
  }
  targ = String(((avetarget.mean()) / 10), 1);
  if (avetarget.mean() < 10) {
    targ = String(" " + targ);
  }
  tft.print((target / 10), 1); //
  tft.setCursor(62, 18);
  tft.print("perf ");//
  value[0] = ((sog / target) * 1000);
  if (value[0] > 400) {
   value[0] = 0;
  }
  if (value[0] < 100) {
    tft.drawRightString("    ", 110, 18, 1);
    tft.drawRightString(String(value[0]), 110, 18, 1);
  }
  else {
    tft.drawRightString(String(value[0]), 110, 18, 1);
  }

  //LIGNE4

  tft.setCursor(0, 27);
  tft.print("sog ");
  tft.print(sog);//
  tft.print("  ");
  tft.setCursor(62, 27);
  tft.print("COG ");
  tft.print((RadToDeg(cog)), 0);
  tft.print("  ");

  tft.setCursor(  124, 27);
  tft.print("twa ");
  tft.print((RadToDeg(twa)), 0); //
  tft.print("  ");
  tft.setCursor(186, 27);
  tft.print("tws ");
  tft.print(tws);//
  tft.print("  ");
  //LIGNE5 36

  tft.setCursor(0, 36);
  tft.print("Mean ");
  tft.print(targ);
  tft.print("  ");
  //tft.print("HDG ");
  //tft.print((RadToDeg(head)), 0); //
  tft.setCursor(62, 36);
  tft.print("twd ");
  tft.print((RadToDeg(twd)), 0);//
  tft.print("  ");

 
  tft.setCursor(124, 36);
  tft.print("aws ");
  tft.print(aws);
  tft.print("  ");
  tft.setCursor(186, 36);
  tft.print("awa ");
  if ((RadToDeg(awa)) > 180) {
    tft.print(" - ");
    tft.print((360 - (RadToDeg(awa))), 0);
    tft.print("  ");
  }
  else {
    tft.print(" ");
    tft.print((RadToDeg(awa)), 0);
    tft.print("  ");
  }
  // affichage  perf meter--------------------------------
  xpos =  0, ypos = 140, gap = 15, radius = 80;
  reading = value[0];
  ringMeter(reading, 0, 130, xpos, ypos, radius, "perf", BLUE2RED); // Draw analogue meter
  //_______________affichage target
  tft.setTextSize(1);
  tft.setTextPadding(3 * 14); // Allow for 3 digits each 14 pixels wide
  tft.drawString(targ, xpos + radius, ypos + radius + radius - 15, 4); // Value in middle
  tft.setTextPadding(0);
  tft.drawString("target", xpos + radius, ypos + radius + radius, 2); // Units display
  
  
  
  //_________________affichage sog
  tft.setTextSize(1);
  xpos =  160, ypos = 60;
  tft.setTextPadding(3 * 34); // Allow for 3 digits each 34 pixels wide
  tft.drawString(String(sog,1), xpos, ypos+40 , 6); // Value in middle
  tft.setTextPadding(0);
  tft.drawString("sog", xpos, ypos, 4); // Units display
//_________________affichage tws
  tft.setTextSize(1);
  xpos =  50, ypos = 60;
  tft.setTextPadding(3 * 34); // Allow for 3 digits each 34 pixels wide
  tft.drawString(String(tws,1), xpos, ypos+40 , 6); // Value in middle
  tft.setTextPadding(0);
  tft.drawString("tws", xpos, ypos, 4); // Units display
}

void get_true(double C_sog, double C_cog, double C_aws, double C_heading, double C_awa, double & twd, double & tws, double & twa) {
  // calcul twa tws
  // Convert apparent wind speed to units of ships speed.
  C_aws = C_aws / C_sog;
  if ( C_heading == 0) {
    C_heading = C_cog;
  }
  // Calculate true heading diff and true wind speed
  double tanAlpha = (sin(C_awa) / (C_aws - cos(C_awa)));
  double alpha = atan(tanAlpha);
  double tdiff = (C_awa + alpha);
  double tspeed = sin(C_awa) / sin(alpha);

  twd = tdiff + C_heading ;// , 2
  if ( RadToDeg(twd) > 360) {
    twd -= DegToRad(360);
  }
  //tanAlpha: _.round tanAlpha, 5
  //alpha: _.round alpha, 5
  twa = tdiff;
  //twd: twd
  //twsr: _.round tspeed, 2
  tws = (tspeed * C_sog); //, 2
if (tws<0){ 
  tws=-tws;
  }
  // fin calcul twa tws
}


int FindTarget (int rawtws, int rawtwa) { //rawtws en kts x10 rawtwa en degrés int pour réduire la charge du processeur
  int itws;
  int itwa;
  if (rawtwa > 180) {  //
    rawtwa = (360 - rawtwa);
  }
  //rawtws = rawtws * 10;
  //modification tws pour correspondre au tableau regele de trois
  // les entrées du fichier pol n'etant pas lineaires
  if (rawtws <= 40 ) {
    itws = rawtws / 4 ;
    if (rawtws % 4 > 5) {
      itws = itws + 1;
    }
  }

  else if (rawtws <= 200 ) {
    itws = 10 + ((rawtws - 40) / 2) ;
    if ((rawtws - 40) % 2 > 5) {
      itws = itws + 1;
    }
  }
  else if (rawtws <= 600 ) {
    itws = 90 + (rawtws - 200) / 5 ;
    if ((rawtws - 200) % 5 > 5) {
      itws = itws + 1;
    }
  }

  //modification twa pour correspondre au tableau
  if (rawtwa <= 25 ) {
    itwa = rawtwa ;
  }
  else if (rawtwa <= 32 ) {
    itwa = 25 + ((rawtwa - 25) / 7 * 5) ;
  }
  else if (rawtwa <= 36 ) {
    itwa = 30 + ((rawtwa - 32) / 4 * 5) ;
  }
  else if (rawtwa <= 40 ) {
    itwa = 35 + ((rawtwa - 36) / 4 * 5) ;
  }
  else if (rawtwa <= 45 ) {
    itwa = rawtwa;
  }
  else if (rawtwa <= 52 ) {
    itwa = 45 + ((rawtwa - 45) / 7 * 5) ;
  }
  else if (rawtwa <= 60 ) {
    itwa = 50 + ((rawtwa - 52) / 8 * 5) ;
  }
  else if (rawtwa <= 180 ) {
    itwa = 55 + ((rawtwa - 60) / 10 * 5) ;
  }

  // Find right table indices to work on
  int itwsIL, itwsIH;
  int itwaIL, itwaIH;
  long itwsOffset, itwaOffset;

  if (itws < itwsMin) {
    itwsIL = itwsIH = 0;
    itwsOffset = 0;
  }
  else if (itws > itwsMax) {
    itwsIL = itwsIH = itwsSteps - 1;
    itwsOffset = 0;
  }
  else {
    itwsIL = (itws - itwsMin) / itwsFactor;
    itwsIH = itwsIL + 1;
    itwsOffset = (itws - itwsMin) % itwsFactor;
  }

  if (itwa < itwaMin) {
    itwaIL = itwaIH = 0;
    itwaOffset = 0;
  }
  else if (itwa > itwaMax) {
    itwaIL = itwaIH = itwaSteps - 1;
    itwaOffset = 0;
  }
  else {
    itwaIL = (itwa - itwaMin) / itwaFactor;
    itwaIH = itwaIL + 1;
    itwaOffset = (itwa - itwaMin) % itwaFactor;
  }

  // Interpolate the values
  long fLL = Target[itwaIL][itwsIL];
  delayMicroseconds (0);
  long fLH = Target[itwaIL][itwsIH];
  delayMicroseconds (0);
  long fHL = Target[itwaIH][itwsIL];
  delayMicroseconds (0);
  long fHH = Target[itwaIH][itwsIH];

  return (((fLL * (itwsFactor - itwsOffset)
            + fLH * itwsOffset) * (itwaFactor - itwaOffset))
          + ((fHL * (itwsFactor - itwsOffset)
              + fHH * itwsOffset) * itwaOffset))
         / (itwsFactor * itwaFactor);
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

// #########################################################################
//  Draw the meter on the screen, returns x coord of righthand side
// #########################################################################
int ringMeter(int value, int vmin, int vmax, int x, int y, int r, const char *units, byte scheme)
{
  // Minimum value of r is about 52 before value text intrudes on ring
  // drawing the text first is an option

  x += r; y += r;   // Calculate coords of centre of ring

  int w = r / 3;    // Width of outer ring is 1/4 of radius

  int angle = 150;  // Half the sweep angle of meter (300 degrees)

  int v = map(value, vmin, vmax, -angle, angle); // Map the value to an angle v

  byte seg = 3; // Segments are 3 degrees wide = 100 segments for 300 degrees
  byte inc = 6; // Draw segments every 3 degrees, increase to 6 for segmented ring

  // Variable to save "value" text colour from scheme and set default
  int colour = TFT_BLUE;

  // Draw colour blocks every inc degrees
  for (int i = -angle + inc / 2; i < angle - inc / 2; i += inc) {
    // Calculate pair of coordinates for segment start
    float sx = cos((i - 90) * 0.0174532925);
    float sy = sin((i - 90) * 0.0174532925);
    uint16_t x0 = sx * (r - w) + x;
    uint16_t y0 = sy * (r - w) + y;
    uint16_t x1 = sx * r + x;
    uint16_t y1 = sy * r + y;

    // Calculate pair of coordinates for segment end
    float sx2 = cos((i + seg - 90) * 0.0174532925);
    float sy2 = sin((i + seg - 90) * 0.0174532925);
    int x2 = sx2 * (r - w) + x;
    int y2 = sy2 * (r - w) + y;
    int x3 = sx2 * r + x;
    int y3 = sy2 * r + y;

    if (i < v) { // Fill in coloured segments with 2 triangles
      switch (scheme) {
        case 0: colour = TFT_RED; break; // Fixed colour
        case 1: colour = TFT_GREEN; break; // Fixed colour
        case 2: colour = TFT_BLUE; break; // Fixed colour
        case 3: colour = rainbow(map(i, -angle, angle, 0, 127)); break; // Full spectrum blue to red
        case 4: colour = rainbow(map(i, -angle, angle, 70, 127)); break; // Green to red (high temperature etc)
        case 5: colour = rainbow(map(i, -angle, angle, 127, 63)); break; // Red to green (low battery etc)
        default: colour = TFT_BLUE; break; // Fixed colour
      }
      tft.fillTriangle(x0, y0, x1, y1, x2, y2, colour);
      tft.fillTriangle(x1, y1, x2, y2, x3, y3, colour);
      //text_colour = colour; // Save the last colour drawn
    }
    else // Fill in blank segments
    {
      tft.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_GREY);
      tft.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_GREY);
    }
  }
  // Convert value to a string
  char buf[10];
  byte len = 3; if (value > 999) len = 5;
  dtostrf(value, len, 0, buf);
  // buf[len] = ' '; buf[len+1] = 0; // Add blanking space and terminator, helps to centre text too!
  // Set the text colour to default
  tft.setTextSize(1);

  if (value < vmin || value > vmax) {
    drawAlert(x, y + 90, 50, 1);
  }
  else {
    drawAlert(x, y + 90, 50, 0);
  }

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  // Uncomment next line to set the text colour to the last segment value!
  tft.setTextColor(colour, TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  // Print value, if the meter is large then use big font 8, othewise use 4
  if (r > 84) {
    tft.setTextPadding(55 * 3); // Allow for 3 digits each 55 pixels wide
    tft.drawString(buf, x, y, 8); // Value in middle
    tft.setTextColor(TFT_WHITE, TFT_BLACK);

  }
  else {
    tft.setTextPadding(3 * 14); // Allow for 3 digits each 14 pixels wide
    tft.drawString(buf, x, y, 4); // Value in middle
  }
  tft.setTextSize(1);
  tft.setTextPadding(0);
  // Print units, if the meter is large then use big font 4, othewise use 2
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  if (r > 84) tft.drawString(units, x, y + 60, 4); // Units display
  else tft.drawString(units, x, y + 15, 2); // Units display

  // Calculate and return right hand side x coordinate
  return x + r;
}

void drawAlert(int x, int y , int side, boolean draw)
{
  if (draw && !range_error) {
    drawIcon(alert, x - alertWidth / 2, y - alertHeight / 2, alertWidth, alertHeight);
    range_error = 1;
  }
  else if (!draw) {
    tft.fillRect(x - alertWidth / 2, y - alertHeight / 2, alertWidth, alertHeight, TFT_BLACK);
    range_error = 0;
  }
}

// #########################################################################
// Return a 16 bit rainbow colour
// #########################################################################
unsigned int rainbow(byte value)
{
  // Value is expected to be in range 0-127
  // The value is converted to a spectrum colour from 0 = blue through to 127 = red

  byte red = 0; // Red is the top 5 bits of a 16 bit colour value
  byte green = 0;// Green is the middle 6 bits
  byte blue = 0; // Blue is the bottom 5 bits

  byte quadrant = value / 32;

  if (quadrant == 0) {
    blue = 31;
    green = 2 * (value % 32);
    red = 0;
  }
  if (quadrant == 1) {
    blue = 31 - (value % 32);
    green = 63;
    red = 0;
  }
  if (quadrant == 2) {
    blue = 0;
    green = 63;
    red = value % 32;
  }
  if (quadrant == 3) {
    blue = 0;
    green = 63 - 2 * (value % 32);
    red = 31;
  }
  return (red << 11) + (green << 5) + blue;
}



//====================================================================================
// This is the function to draw the icon stored as an array in program memory (FLASH)
//====================================================================================

// To speed up rendering we use a 64 pixel buffer
#define BUFF_SIZE 64

// Draw array "icon" of defined width and height at coordinate x,y
// Maximum icon size is 255x255 pixels to avoid integer overflow

void drawIcon(const unsigned short * icon, int16_t x, int16_t y, int8_t width, int8_t height) {

  uint16_t  pix_buffer[BUFF_SIZE];   // Pixel buffer (16 bits per pixel)

  tft.startWrite();

  // Set up a window the right size to stream pixels into
  tft.setAddrWindow(x, y, width, height);

  // Work out the number whole buffers to send
  uint16_t nb = ((uint16_t)height * width) / BUFF_SIZE;

  // Fill and send "nb" buffers to TFT
  for (int i = 0; i < nb; i++) {
    for (int j = 0; j < BUFF_SIZE; j++) {
      pix_buffer[j] = pgm_read_word(&icon[i * BUFF_SIZE + j]);
    }
    tft.pushColors(pix_buffer, BUFF_SIZE);
  }

  // Work out number of pixels not yet sent
  uint16_t np = ((uint16_t)height * width) % BUFF_SIZE;

  // Send any partial buffer left over
  if (np) {
    for (int i = 0; i < np; i++) pix_buffer[i] = pgm_read_word(&icon[nb * BUFF_SIZE + i]);
    tft.pushColors(pix_buffer, np);
  }

  tft.endWrite();
}
