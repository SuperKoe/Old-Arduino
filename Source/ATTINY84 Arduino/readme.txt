attiny84-02 install instructions 02/06/2010
============================================================


The release of Arduino 0018 has eased the intergration of third part boards, here are
the instructions of how to add support for the attiny into 0018.

It is best to start with a fresh install of Arduino IDE 0018.

Unzip attiny84-02, you will have a folder called attiny84

For the purpose of these instructions, i have the IDE installed at C:\Arduino0018\

open the Arduino IDE folder and then the hardware folder, there will be 2 folders there
already, "Arduino" and "tools"

C:\
____Arduino0018
________________hardware
_________________________arduino
_________________________tools



copy (drag n drop) the attiny84 folder into the hardware folder


C:\
____Arduino0018
________________hardware
_________________________arduino
_________________________tools
_________________________attiny84



Run Arduino IDE, click on Tool then boards and you will see ATtiny84 is now listed, 
Select ATtiny84 as your board and you're sorted.

If you dont have a ISP programmer you can use the arduino Examples ArduinoISP
Upload that sketch to a normal arduino, see http://arduino.cc/en/Tutorial/ArduinoISP
And Connect arduinopin 13 to CLK, 12 to MISO, 11 to MOSI and 10 to RESET
Also hook up the 5V+ and GND. For those line on a attiny, check the datasheet @ atmel.com
for the 14pdip it's:
pin 1 5V
pin 4 RESET (Needs a 10K resistor in parrallel with 5V+)
pin 7 MOSI
pin 8 MISO
pin 9 CLK
pin 14 GND

                     +-\/-+
               VCC  1|    |14  GND
          (D0) PB0  2|    |13  AREF
          (D1) PB1  3|    |12  PA1 (D9)
             RESET  4|    |11  PA2 (D8)
INT0  PWM (D2) PB2  5|    |10  PA3 (D7)
      PWM (D3) PA7  6|    |9   PA4 (D6)
      PWM (D4) PA6  7|    |8   PA5 (D5) PWM
                     +----+

Uploading sketch's to the attiny84 you have to use th arduinoISP.

If the chip is brand new, you have to set the fuses correct, in this case, a 8Mhz internal clock.
The burnbootloader process in the arduino sets the fuses, and uploads the bootloader.
But the attiny84 doesnt have a bootloader...so i made a fake bootloader.
It uploades the blink sketch on pin 2.

Now you can upload to the attiny84 without bootloader :)

Instructions are modifyed sanguino's instructions :)
