# Esp32-polardisplay
This code is not perfect, i'm newbie, but it works.
polar display is the first module
i will add wireless sensor in the future
the server is the central node for connection to nmea2000, and wireless dispatching
it's calculating target speed from polar data, the data are sent to external display
data can be send to a fake nmea sentence as rpm or tank

Remote Display : to display the calculated performance from the wind data coming from NMEA2000 and the polar file of my boat

There is two display one is a transflective 1.6"LCD MODULE small but CHEAP and sunlight readable
connection  led IO4
            sck IO18
            SDA IO23
            AO/DC IO17
            RST IO16
            CS IO5 
the other is a big transflective lcd 20X4 I2C


The server is esp32 using built in can with CJMCU-230 SN65HVD230 CAN Bus Transceiver Module
(SN65HVD230 need pull up and pull down resitor on tx and rx to work)
CRX->pin17 CTX->pin5
BMP280   SCK->PIN15 SDA->4
OLED is just for debbugging a 4 inch display is comming for future add on(fuel an water level etc). 
Server is connected via a router but can be set as access point.



Future add : 
- send temp and pressure to nmea2000           
       - fuel flow module
              - chain meter module
              - water dessalator flow meter module
              - engine rpm
              
              
 
