// pour l'utilisation Du NMEA2000 https://github.com/ttlappalainen/NMEA2000
// pour l'interpolation d'une table  https://forum.arduino.cc/index.php?topic=8484.15
// pour le serveur client esp32 https://randomnerdtutorials.com/esp32-client-server-wi-fi/
// pour le calcul de la vitesse cible https://github.com/cape-io/true-wind/blob/master/true-wind.coffee
/*BMP280 Barometric/Temp sensor with OLED display
   I2C
   CHANGE ADRESS to 76 IN <Adafruit_BMP280.h> IF NECESSARY
   Wiring Diagram:
   SDA from BMP and OLED to A4 Pin on Arduino Uno
   SCL from BMP and OLED to A5 Pin
   GND to GND
   VCC from BMP and OLED to 3V
*/
//ESP32 CAN
#define ESP32_CAN_TX_PIN GPIO_NUM_17
#define ESP32_CAN_RX_PIN GPIO_NUM_5


//MEGA  CAN
//#define N2k_SPI_CS_PIN 53  // Pin for SPI Can Select
//#define N2k_CAN_INT_PIN 21 // Use interrupt  and it is connected to pin 21
//#define USE_MCP_CAN_CLOCK_SET 16  // possible values 8 for 8Mhz and 16 for 16 Mhz clock
//// #define USE_DUE_CAN 1
//#define N2K_SOURCE 15

#include <Arduino.h>
#include <NMEA2000_CAN.h>  // This will automatically choose right CAN library and create suitable NMEA2000 object
#include <N2kMsg.h>
#include <NMEA2000.h>
#include <N2kMessages.h>

#include "ESPAsyncWebServer.h"
#include "WiFi.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>  //POUR BMP
#include <Adafruit_BMP280.h> //POUR BMP
#include <Adafruit_SSD1306.h> //POUR OLED
#include <Average.h>
#include <Flash.h>
//OLED pins
#define OLED_SDA 4
#define OLED_SCL 15
#define OLED_RST 16
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Set your network
const char* ssid = "yourssid";
const char* password = "yourpassword";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

//POLAIRE
const int itwsSteps = 18; // Number of entries for itws l'idée vient d'ici https://forum.arduino.cc/index.php?topic=8484.15
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

// TIMER
unsigned long previousMillis = 0;
const long interval = 1000; // INTERVALLE RAFRAICHISSEMENT

String readTemp() {
  return String(float(bmp.readTemperature()), 1);
}
String readPres() {
  return String(float(bmp.readPressure() / 100.0F), 1);
}
String readtarget() {
  return String(targ);
}
String readtwa() {
  return String(twa);
}
String readtws() {
  return String(tws);
}


void startOLED() {
  //reset OLED display via software
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(20);
  digitalWrite(OLED_RST, HIGH);

  //initialize OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false)) { // Address 0x3C for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }
}

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

void setup() {

  Serial.begin(115200);
  startOLED();

  display.clearDisplay();
  display.setTextColor(WHITE);
  display.println( F("ServeurPolaire"));
  display.display();

  Serial.println(F("BMP280 Baro Sensor et performance"));

  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    display.clearDisplay();
    display.setCursor(0, 20);
    display.println( F("BMP SENSOR"));
    display.setCursor(0, 50);
    display.println( F("NOT FOUND!"));
    display.display();

    while (1);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  bmp_temp->printSensorDetails();

  // Do not forward bus messages at all
  //NMEA2000.EnableForward(false);
  NMEA2000.SetMsgHandler(HandleNMEA2000Msg);
  NMEA2000.Open();

  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Print local IP address and start web server

  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  //delay(5000);

  // pour acceder aux données depuis un client
  server.on("/temperature", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain",  readTemp().c_str());
  });
  server.on("/pressure", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", readPres().c_str());
  });
  server.on("/target", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", readtarget().c_str());
  });
  server.on("/twa", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", readtwa().c_str());
  });
  server.on("/tws", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", readtws().c_str());
  });


  server.begin();

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
  double twa, twd, tws;
  NMEA2000.ParseMessages();
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    i = i + 1;

    display.clearDisplay();

    double temp = bmp.readTemperature();
    Serial.print(F("Temperature = "));
    Serial.print(temp, 1); //     ,# sets the number of decimal places
    Serial.println(" *C");



    //LIGNE1
    display.setCursor(0, 0);
    display.print( F("Temp: "));
    display.setCursor(62, 0);
    display.print(temp, 1); //    ,# sets the number of decimal places
    display.print(F("   C"));
    display.setCursor(92, 0);
    display.print((char)247); // degree symbol

    //LIGNE2
    double pressure = bmp.readPressure() / 100 ;
    Serial.print(pressure, 1); //    ,# sets the number of decimal places
    Serial.println(F(" mB"));
    display.setCursor(0, 9);
    display.print( F("Press: "));
    display.setCursor(62, 9);
    display.print(pressure, 1); //   ,# sets the number of decimal places
    display.print(F(" mB"));

    //LIGNE3
    display.setCursor(0, 18);
    display.print("TgtS ");//
    if (aws == 0 && awa == 0) {
      target = 0;
    }
    else {
      get_true(sog, cog, aws, head, awa, twd, tws, twa);
      target = (FindTarget((tws * 10), RadToDeg(twa)));
      avetarget.push(target);
      // ancie double target = (FindTarget((aws * 10), RadToDeg(awa)));
    }
    targ = String(((avetarget.mean()) / 10), 1);
    if (avetarget.mean() < 10) {
      targ = String(" " + targ);
    }
    display.print((target / 10), 1); //
    display.setCursor(62, 18);
    display.print("perf ");//
    display.print(((sog / target) * 1000), 0);

    //LIGNE4
    display.setCursor(0, 27);
    display.print("sog ");
    display.print(sog);//
    display.setCursor(62, 27);
    display.print("COG ");
    display.print((RadToDeg(cog)), 0);


    //LIGNE5 36
    display.setCursor(0, 36);
    display.print("Mean ");
    display.print(targ);
    //display.print("HDG ");
    //display.print((RadToDeg(head)), 0); //
    display.setCursor(62, 36);
    display.print("twd ");
    display.print((RadToDeg(twd)), 0);//

    //LIGNE6 36
    display.setCursor(0, 45);
    display.print("twa ");
    display.print((RadToDeg(twa)), 0); //
    display.setCursor(62, 45);
    display.print("tws ");
    display.print(tws);//

    //LIGNE7 54
    display.setCursor(0, 54);
    display.print("aws ");
    display.print(aws);
    display.setCursor(62, 54);
    display.print("awa ");
    if ((RadToDeg(awa)) > 180) {
      display.print(" - ");
      display.print((360 - (RadToDeg(awa))), 0);
    }
    else {
      display.print(" ");
      display.print((RadToDeg(awa)), 0);
    }
    display.display();
  }
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
