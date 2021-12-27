#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <UDP.h>
#include <Average.h>
#include <Preferences.h>

Average<float> ave(10);
//Horloge INTERNET NTP
WiFiUDP ntpUDP;

NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", 3600, 60000);

//Variables Web
const char* SSID = "yourssid";
const char* PASSWORD = "yourpass";
// Set web server port number to 80
AsyncWebServer server(80);
//FIN Variables Web

//Variables Reed
#define DEBOUNCE_TIME 250
#define PIN_BUTTON 23
#define PIN_LED 2

//OLED pins
#define OLED_SDA 4
#define OLED_SCL 15
#define OLED_RST 16
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

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
String LastSaveTime = "00/00/00";


int button_count = 0;
volatile int DebounceTimer = 0;
int ETAT;
//FIN variables Reed

// Variables Débit
#define SENSOR  27


long previousMillisSaveData = 0;
long intervalSaveData = 600000;

long currentMillis = 0;
long previousMillis = 0;
float interval = 1000.0;

boolean Saisie1;
boolean Saisie2;
boolean Saisie3;
float calibrationFactor = 8;/* 16    16,14  16 15.74 */
float K = 496;
volatile byte pulseCount;
volatile byte ReedPushCounter;
byte Pulse1Int = 0;

float flowRate;
float flowMilliLitres;
float totalMilliLitres;
float TotalCuve ;
float Reste;
float Correction;

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

//FIN Variables Débit

//FONCTION WEB SAISIE PARAMETRE
const char* PARAM_INPUT_1 = "input1";
const char* PARAM_INPUT_2 = "input2";
const char* PARAM_INPUT_3 = "input3";

String inputMessage;

// HTML web page to handle 3 input fields (input1, input2, input3)
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>ESP Input Form</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  </head><body>
  <form action="/get" >
   NouveauReste___ : <input type="number" name="input1"> 
    <input type="submit" value="Submit">
  </form><br>
  <form action="/get">
    RESET PROD : <input type="number" name="input2">
    <input type="submit" value="Submit">
  </form><br>
  <form action="/get">
    K_________K843/496 : <input type="number" name="input3">
    <input type="submit" value="Submit">
  </form>
</body></html>)rawliteral";

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}
//FONCTION WEB (convertion de la variable en chaine pour transmettre en GET
String ReadCounterReed() {
  return String(button_count);
}

String ReadDebit() {
  return String(ave.mean());//ave.mean()(flowRate)
}

String ReadSumVolume() {
  return String(TotalCuve);
}

String ReadReste() {
  return String(Reste);
}




//FIN FONCTION WEB

void initWiFi() {
  // Connect to Wi-Fi network with SSID and PASSWORD
  Serial.print("Connecting to ");
  Serial.println(SSID);
  WiFi.begin(SSID, PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Print local IP address
  Serial.println("");
  Serial.println("WiFi connected at IP address:");
  Serial.println(WiFi.localIP());

  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setCursor(10, 10);
  display.println(WiFi.localIP());
  display.display();
  // Fonction Reed La fonction est placee dans la RAM de l ESP32.
}

void IRAM_ATTR buttonpressed() {
  if ( millis() - DEBOUNCE_TIME  >= DebounceTimer ) {
    DebounceTimer = millis();
    button_count += 1;
    ETAT = button_count % 2;
    //Serial.printf("Button has been pressed %u times\n", button_count);
  }
}
// FIN Fonction Reed


// Fonctions Débimetre
void IRAM_ATTR pulseCounter()
{
  pulseCount++;
}

//FIN fonctions débimetre
//lecture données stockées
//initialisation preferences stockage flash
Preferences preferences;

void SaveData() {
  preferences.begin("compteur", false);
  if (TotalCuve != preferences.getFloat("Production", 0.0)) {
    // Store the data to the Preferences
    preferences.putFloat("Production", TotalCuve);
    Serial.print(" PRODUCTION datasaved");
  }
  if (button_count != preferences.getFloat("Conso", 0.0)) {
    preferences.putFloat("Conso", button_count);
    Serial.print(" CONSO datasaved");
  }
  if (Correction != preferences.getFloat("Correction", 0.0)) {
    preferences.putFloat("Correction", Correction);
    Serial.print(" Correction datasaved");
  }
  if (calibrationFactor != preferences.getFloat("Calibration", 0.0)) {
    preferences.putFloat("Calibration", button_count);
    Serial.print(" Calibration datasaved");
  }
  if (K != preferences.getFloat("K", 0.0)) {
    preferences.putFloat("K", K);
    Serial.print(" K datasaved");
  }


  // Close the Preferences
  preferences.end();

  LastSaveTime = timeClient.getFormattedTime();
}

void setup()
{
  Serial.begin(115200);

  //lecture données stockées
  preferences.begin("compteur", false);
  TotalCuve = preferences.getFloat("Production", 0.0);
  button_count = preferences.getFloat("Conso", 0.0);
  Correction = preferences.getFloat("Correction", 0.0);
  K = preferences.getFloat("K", 0.0);
  calibrationFactor = K/60;
  totalMilliLitres = TotalCuve * K;
  Serial.println("donnes recuperees Preferences");
  // Close the Preferences
  preferences.end();


  pulseCount = 0;
  flowRate = 0.0;
  flowMilliLitres = 0;
  //totalMilliLitres = 0;
  previousMillis = 0;

  startOLED();

  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setCursor(40, 20);
  display.println( F("LUNE"));
  display.setCursor(20, 45);
  display.println( F("COMPTEUR CUVE"));
  display.display();


  display.clearDisplay();
  Serial.println("LUNE_COMPTEUR_CUVE");

  //config reed
  pinMode(PIN_BUTTON, INPUT);
  attachInterrupt(PIN_BUTTON, buttonpressed, FALLING);

  // Configure la sortie de la LED pour le comptage Reed
  pinMode(PIN_LED, OUTPUT);

  initWiFi();
  //Config Web



  server.on("/CompteurReed", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain",  ReadCounterReed().c_str());
  });

  server.on("/Debit", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain",  ReadDebit().c_str());
  });

  server.on("/Volume", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain",  ReadSumVolume().c_str());
  });

  server.on("/Reste", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain",  ReadReste().c_str());
  });

  // Send web page with input fields to client
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/html", index_html);
  });
  // DEBUT SAISIE
  // Send a GET request to <ESP_IP>/get?input1=<inputMessage>
  server.on("/get", HTTP_GET, [] (AsyncWebServerRequest * request) {
    // String inputMessage;
    String inputParam;
    // GET input1 value on <ESP_IP>/get?input1=<inputMessage>
    if (request->hasParam(PARAM_INPUT_1)) {
      inputMessage = request->getParam(PARAM_INPUT_1)->value();
      inputParam = PARAM_INPUT_1;
      Saisie1 = true ;
    }
    // GET input2 value on <ESP_IP>/get?input2=<inputMessage>
    else if (request->hasParam(PARAM_INPUT_2)) {
      inputMessage = request->getParam(PARAM_INPUT_2)->value();
      inputParam = PARAM_INPUT_2;
      Saisie2 = true ;
    }
    // GET input3 value on <ESP_IP>/get?input3=<inputMessage>
    else if (request->hasParam(PARAM_INPUT_3)) {
      inputMessage = request->getParam(PARAM_INPUT_3)->value();
      inputParam = PARAM_INPUT_3;
      Saisie3 = true ;
    }
    else {
      inputMessage = "No message sent";
      inputParam = "none";
    }
    Serial.println(inputMessage);
    request->send(200, "text/html", "HTTP GET request sent to your ESP on input field ("
                  + inputParam + ") with value: " + inputMessage +
                  "<br><a href=\"/\">Return to Home Page</a>");
  });
  server.onNotFound(notFound);

  // Start Web Server
  server.begin();
  //FIN Config Web

  // Démarrage du client NTP - Start NTP client
  timeClient.begin();
  // Met à jour l'heure toutes les 10 secondes | update time every 10 secondes
  timeClient.setTimeOffset(7200);
  timeClient.update();


  //Config Débimetre
  pinMode(SENSOR, INPUT);


  attachInterrupt(SENSOR, pulseCounter, FALLING);
  //FIN Config Débimetre

}


void loop() {


  if (Saisie1 == true) {
    Correction = inputMessage.toFloat() - TotalCuve + button_count ;
    SaveData();
    Serial.println( Correction, 1);
    Saisie1 = false;
  }
  if (Saisie2 == true) {
     TotalCuve = button_count;
      totalMilliLitres = TotalCuve * K;
    Correction = Reste - TotalCuve + button_count ;
    SaveData();
        Saisie2 = false;
  }
  if (Saisie3 == true) {
    K = inputMessage.toFloat() ;
    TotalCuve = totalMilliLitres / K;
    calibrationFactor = K/60;
    Correction = Reste - TotalCuve + button_count ;
    SaveData();
    Serial.println( K, 1);
    Saisie3 = false;
  }

  currentMillis = millis();
  if (currentMillis - previousMillisSaveData > intervalSaveData) {
    SaveData();
    if (WiFi.status() != WL_CONNECTED)  {
      Serial.println("Reconnecting to WiFi...");
      WiFi.disconnect();
      WiFi.reconnect();
    }

    previousMillisSaveData = millis();
  }

  currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    Pulse1Int = pulseCount;
    if (Pulse1Int > 40) {
      Pulse1Int = 0;
    }

    pulseCount = 0;

    flowRate = ((((interval / (currentMillis - previousMillis))) * Pulse1Int) / calibrationFactor) / (interval / 1000.0);
    previousMillis = millis();
    ave.push(flowRate);

    // Add the millilitres passed in this second to the cumulative total
    totalMilliLitres += Pulse1Int;
    TotalCuve = totalMilliLitres / K;
    Reste = TotalCuve - button_count + Correction;

    display.clearDisplay();

    Serial.print(timeClient.getFormattedTime());

    display.setCursor(0, 0);
    display.println(WiFi.localIP());
    display.setCursor(74, 10);
    display.print(LastSaveTime);
    
    display.setCursor(0, 10);
    display.print(timeClient.getFormattedTime());
    Serial.print("  Débit : ");
    Serial.print(flowRate, 1); // Print the integer part of the variable
    Serial.print("L/mn ");
    Serial.print("\t");
    Serial.print("Mean:   "); Serial.print(ave.mean());


    display.setCursor(0, 20);
    display.print("Debit : ");
    display.print(ave.mean());
    display.print("L/mn ");
    display.print(Pulse1Int);


    // Print the cumulative total of litres flowed since starting


    Serial.print("  Production : ");
    Serial.print(TotalCuve, 2);
    Serial.print(" L ");

    display.setCursor(0, 30);
    display.print("Prod ");
    display.print(TotalCuve, 2);
    display.print(" L");


    Serial.print(" Consommation : ");
    Serial.print(ReadCounterReed());
    Serial.print("L ");

    display.setCursor(0, 40);
    display.print("Conso ");
    display.print(ReadCounterReed());
    display.print(" L");


    Serial.print(" Debit Brut en Pulsation : ");
    Serial.println(Pulse1Int);

    display.setCursor(0, 50);
    display.print("Reste ");
    display.print(Reste, 0);
    display.print(" L");

    display.print(" cal ");
    display.print(K, 0);
   

    display.display();



  }
}
