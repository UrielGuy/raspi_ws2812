# raspi_ws2812
A kernel module able to run as many WS2812 strips as there are GPIO pins on Raspberry Pi Zero. 
This kernael module + library to use it will allow you to specify as many LED strips as raspberry pi have GPIO pins (27), and connect up to 1024 LEDs to each. The kernel module will recieve a frame over a char device, freeze all interrupts on the raspberry pi, basically halting the kernel. now it will bitbang out the WS2812 frames on all strips at the same time using the GPIO registers, and resume the kernel. Time of the kernel being frozen relative to the LED strip length, so I would recommend many short strips and not few long ones.

## Requirements
This module requires a Raspberry Pi 1/1B/Zero/ZeroW/ZeroWH. It will NOT run on Raspberry Pi 2/3, and wii probably cause it to crash and burn. Maybe even literally.
Another thing you will have to do is provide logic level transition to all of the strips - Raspberry Pis will output 3.3V while WS2812 strips require 5V inputs. I like using 74HCT245 chips for that. Adafruit have some suggestions here: https://learn.adafruit.com/neopixels-on-raspberry-pi/wiring

## Building and running the kernel module
* Install Raspbian Stretch (Might work on others, haven't tried) 
* Update you installation, and get the kernel headers: `sudo apt-get update && sudo apt-get upgrade -y && sudo apt-get install raspberrypi-kernel-headers -y`
* Clone this repo: `git clone https://github.com/UrielGuy/raspi_ws2812.git`
* Go to the kernel module folder and build it: `cd raspi_ws2812/ws2812_rpi0_lkm ; make`
* Congratulation! if everything wokred out you have a kernel module compiled. now just run `insmod ./ws2812_rpi0_lkm.ko`
* To check that the module is running, run `dmesg` and see some messages at the botton marking success. you can also see the `/dev/ws2812_frame` file on your system

## Using the code
This project provides a C++ Header file containing all you need to use the project. It is in the lkm_client folder inside the repo and fairly well documented. 

Here is a minimal example:
```
#include "raspi_ws2812.h"

// Support 100 LED strips on GPIO pins 21, 5, 6
raspi_ws2812 LEDs(100, { 21,5,6 });

int main(void) {
  while (true) {
    // Clear the display
    LEDs.Clear();
		for (uint32_t led = 0; led < LEDs.max_strip_length()-3; led++)
		{
      // Set pixels so that each strip has its own color and LEDs seem to move down the strip
      // Note that strip index is index to the list given in the constructor (0..2) and not a GPIO pin number(5,6,21)
			LEDs.SetPixel((uint8_t)(led % 3), led + (counter), RGB_t(((uint32_t)0xFF) << 8 * (led % 3)));
		}
    // Advance counter in a loop of 3
		counter = (counter + 1) %3;
    // Display the frame
		LEDs.SendFrame();
  }
}
```
There is a slightly larger example in the repo

## Thing to note
* Using this module will change the power governing of the Raspberry Pi to one that provides maximum performace (so that the clock doesn't change while it runs) but it will cause it to heat more and use more battery. If you're already running multiple LED strips I guess the power consumed by the Raspberry Pi Zero isn't your biggest concern
* Sometimes the first bit comes out a bit out of sync, causing the first pixel to glow green. This tends to stop after a while. 
* Stopping the Kernel to bitbang out is probably a bad idea. Do not use this on anything critical or life supporting. But you already knew that. 
* This is my first ever kernel module, and I can find at least 10 things wrong with it of the top of my head (concurrency of a few calls from different processes, parameters changing during frame building, generally not thread safe in ANY way). I'm happy to improve if there's an actual need, but for my needs this is good enough for now. 
* When setting/getting a pixel, the strip number is an index to the list given in the constructor and not a GPIO pin. This will make it easier on you to build and control large LED matrices.
