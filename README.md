# BLESpinBike
Sniff i2c from my Lemond GeForce UT Spin Bike and present it as a bluetooth low energy Fitness Machine Service

The bike has a USB port, which can save your workout stats to a USB stick, so I opened up the case, and noticed that the USB stuff is handled by a daughter board with an Atmel AT90USB647 on it. It appeared that the main PCB was connected to the TWI pins of the daughter board, so I put an oscilloscope on the clock and data lines between the main PCB and the daughter board and saw what looked like i2c communications going between the two boards. I then grabbed a spare ARM board I had and programmed it to sniff the i2c comms. 

I then rode the bike whilst simultaneously recording to USB and sniffing the i2c. I then synchronised the two recordings and decoded the comms between the two boards.

So now I had a real-time stream of the workout data. I then ordered a bluetooth module that had an embedded ARM chip, and that arrived early this week. It was 3.3V only whereas the spin bike outputs 5V. A resistor divider didn't work as it loaded the pins too much and the spin bike threw an error, so I plumbed in a line driver/buffer chip that also did level conversion (74ALS541 for those interested)

I programmed the ARM chip to decode the i2c and present a Bluetooth low energy 'Fitness machine' service using the 'Indoor Bike' characteristic. The bike now appears as a source of speed, power, heart rate, cadence and calories consumed.

I've tried it with zwift and kinomap and it works perfectly! It didn't work as FTMS with RGT so I also supported the 'Cycling Power' and 'Cycling Speed and Cadence' services. RGT likes that but doesn't allow connections to both services at once (I put a support ticket into RGT and it's a known issue)

Next step is to support the fitness machine control point protocol, so that apps can control the resistance on the bike, but Zwift doesn't support that through BLE FTMS yet, it does through ANT FEC, which my Bluetooth module also supports, but that's a lot of work to change over.
