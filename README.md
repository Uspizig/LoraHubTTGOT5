# LoraHubTTGOT5
Please keep in mind. There are some Limitations in the TTn network right now:
1. For usage with TTN V3: Pease consider this change to avoid unwanted Downlink messages. https://github.com/mcci-catena/arduino-lmic/issues/713 


(Prerequisite: This is my Hobby: No Guarantee for HW or SW. I do not take any responsibility or will help you. This is: "Take it as it is or leave it".)
DO NOT USE THIS in ANY COMMERCIAL PRODUCTS


A LORA TTN Daughter Board for a TTGO T5 Board (http://s.click.aliexpress.com/e/NPRqgBkG) with BME680, BME280, CCS811 and connection possibilities for several other I²C Boards


You might have seen several E-Ink Boards from Waveshare lately. I came across some Boards from TTGO with an ESP32 lately on Aliexpress.
While experimenting with these Boards I tought: Why not add a LORA TTN Chip to these Boards and some other environment sensors.

So here is my try:

You may find in this Repository:
1. Hardware: KiCad & Gerber Files for 
2. SMT Assembly Files for jlcPCB 
3. Pinning: Pin Description
4. Software: A how to make these things running relatively easy in an Arduino IDE

What you need to do to get the SW Running:
1. Clone this Git: https://github.com/ZinggJM/GxEPD2 and install to your Arduino lib folder
2. Replace the two files with the ones provided in GxEPDReplace
3. Install: https://github.com/thesolarnomad/lora-serialization
4. Clone this Git: https://github.com/mcci-catena/arduino-lmic
5A. Make the modifications mentioned in otta SW
5B. Edidt config in the MCCI-Catena to set according yoru Country.
6. Get an App ID, from Thethingsnetwork
7. Check if your area has LORA TTN Coverage
8. Try out one of the examples.

What you need to get the HW:
- TTGO T5 EPaper Board: https://de.aliexpress.com/item/32869729970.html
- RFM95 Lora Chip: https://de.aliexpress.com/item/32831799362.html
- Antenna Connector SMA: http://s.click.aliexpress.com/e/5oGOCiY0
- (OPTIONAL)8dBi Antenna 868MHz: http://s.click.aliexpress.com/e/KVkmsTGG
- (OPTIONAL)TPL5110 for Low Power: http://s.click.aliexpress.com/e/EA7q1R48
- Spacer-Screws: http://s.click.aliexpress.com/e/KejkugSM

Hub Board:
1. Download the Gerber and BOM Files
2. Upload to JLCPCB
3. Select SMT Assembly
4. Solder 3 Diodes, Antenna Connector and RFM Module on the Board
5. (Optional): Solder Battery Connector and TPL5110 Chip on the Board.
6. I made a mistake on the FET and switched Drain and Source pin. You need to make a air wire to fix this if you want to use the TPL5110 RTC.

What you need to Solder
1. RFM95 Chip
2. Antenna Connector
3. 3x Diodes
(Optional) TPL5110 Chip and Battery Connector

Housings:
https://www.thingiverse.com/thing:3462483
https://www.thingiverse.com/thing:4055993

Useful Equipment.:
- Soldering Iron TS100: http://s.click.aliexpress.com/e/Eq2v9g12
- USB Cable: http://s.click.aliexpress.com/e/NiloJfUM
- Soldering Mat. http://s.click.aliexpress.com/e/NAoYAQ5K

My projects are influenced by Dave(G6EJD) and Andreas Spiess. So do me a favor and watch their videos.

