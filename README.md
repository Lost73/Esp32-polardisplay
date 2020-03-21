# Esp32-polardisplay
polar display is the first module
i want to add more wireless sensor in the future the server is the central node for connection to nmea2000

Remote Display for calculated performance from the wind data coming from NMEA2000
Sorry for this awfull code, i'm newbie, but it works, server is connected via a router but can be set as access point
The display is a transflective 1.6"LCD MODULE small but CHEAP and sunlight readable
connection  led IO4
            sck IO18
            SDA IO23
            AO/DC IO17
            RST IO16
            CS IO5 

The server is esp32 with oled using built in can with CJMCU-230 SN65HVD230 CAN Bus Transceiver Module
(SN65HVD230 need pull up and pull down resitor on tx and rx to work)
CRX->pin17 CTX->pin5
BMP280   SCK->PIN15 SDA->4
OLED is just for debbugging 
Future add :  - send temp and pressure to nmea2000
              - fuel flow module
              - chain meter module
              - water dessalator flow meter module
              
              
 
