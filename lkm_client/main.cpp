#include <iostream>
#include <cstring>
#include <csignal>

#include "raspi_ws2812.h"

raspi_ws2812 LEDs(100, { 21,5,6 });
bool stop{ false };
void HandleSigint(int) { std::cout << "stopping!" << std::endl; stop = true; }

RGB_t ColorWheel(uint32_t index) {
	index %= (256*3);
	uint8_t area = index / 256;
	uint8_t offset = index & 0xFF;
	switch (area) {
	case 0: return RGB_t(0, offset, 255 - offset);
	case 1: return RGB_t(offset, 255 - offset, 0);
	case 2: return RGB_t(255 - offset, 0, offset);
	}
	return RGB_t(0);
}

int main(void) {

	signal(SIGINT, HandleSigint);

	LEDs.Clear();
	int counter = 0;
	while (!stop)
	{
		//LEDs.Clear();
		for (uint32_t led = 1; led < LEDs.max_strip_length()-3; led++)
		{
			for (int i = 0; i < 3; i++) {
				LEDs.SetPixel(0, led, ColorWheel(64 * (((float)counter / 3) + led + i)));
			}
		}
		++counter;
		LEDs.SendFrame();
		//break;
		usleep(20000);
		//std::getchar();
	}
	return 0;
}
