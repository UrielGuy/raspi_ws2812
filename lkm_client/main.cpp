#include <iostream>
#include <cstring>
#include <csignal>

#include "raspi_ws2812.h"

raspi_ws2812 LEDs(100, { 21,5,6 });
bool stop{ false };
void HandleSigint(int) { std::cout << "stopping!" << std::endl; stop = true; }

int main(void) {

	signal(SIGINT, HandleSigint);

	int counter = 0;
	while (!stop)
	{
		LEDs.Clear();
		for (uint32_t led = 0; led < LEDs.max_strip_length()-3; led++)
		{
			LEDs.SetPixel((uint8_t)(led % 3), led + (counter), RGB_t(((uint32_t)0xFF) << 8 * (led % 3)));
		}
		++counter %= 3;
		LEDs.SendFrame();
		//break;
		usleep(300000);
		//std::getchar();
	}
	return 0;
}
