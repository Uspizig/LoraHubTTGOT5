# LoraHubTTGOT5
(Prerequisite: This is my Hobby: No Guarantee for HW or SW. I do not take any responsibility or will help you. This is take it as it is.)
A Daughter Board for a TTGO T5 Board with Lora and BME680, BMW280, CCS811 and several other I²C Boards


You might have seen several E-Ink Boards from Waveshare lately. I came across some Boards from TTGO with an ESP32 lately on Aliexpress.
While experimenting with these Boards I tought: Why not add a LORA TTN Chip to these Boards and some other environment sensors.

So here is my try:

You may find in this Repository:
1. Hardware: KiCad & Gerber Files for 
2. SMT Assembly Files for jlcPCB 
3. Pinning: Pin Description
4. Software: A how to make these things running relatively easy in an Arduino IDE

What you need to do to get the SW Running:
1. Clone this Git: https://github.com/ZinggJM/GxEPD and install to your Arduino lib folder
2. Replace the two files with the ones provided in Software
3. Install: https://github.com/thesolarnomad/lora-serialization
4. Clone this Git: https://github.com/matthijskooijman/arduino-lmic
5. Replace the files mentioned in this Pull Request: https://github.com/matthijskooijman/arduino-lmic/pull/77/files
6. Get an App ID, from Thethingsnetwork
7. Check if your area has LORA TTN Coverage
8. Try out one of the examples.

What you need to get the HW:
1. Download the Gerber and BOM Files
2. Upload to JLCPCB
3. Select SMT Assembly
4. Solder 3 Diodes, Antenna Connector and RFM Module on the Board
5. (Optional): Solder Battery Connector and TPL5110 Chip on the Board.


My projects are influenced by Dave(G6EJD) and Andreas Spiess. So do me a favor and watch their videos.

