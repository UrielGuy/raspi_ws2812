#pragma once
#include <cstdint>
#include <initializer_list>
#include <fstream>
#include <cstring>
#include <iostream>
#include <cassert>
#include <bitset>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#pragma pack (push)
#pragma pack (0)
// Struct for RGB. if your strips have a different order of color, chage the order of b,r and g.
struct RGB_t {
	RGB_t(uint8_t _r, uint8_t _g, uint8_t _b) :b(_b), r(_r), g(_g) {}
	RGB_t(uint32_t _raw) : raw(_raw) {}
	RGB_t() : raw(0) {}
	union {
		struct {
			uint8_t b, r, g;
			uint8_t padding;
		};
		uint32_t raw{ 0 };
	};
};

std::ostream &operator<<(std::ostream &os, RGB_t const &w) {
	os.width(2);
	os.fill('0');
	return os << std::hex << "R: 0x" << w.r << " G: 0x" << w.g << " B: 0x" << w.b;
}
#pragma pack (pop)

class raspi_ws2812
{
public: 
	raspi_ws2812(uint32_t max_strip_length, std::initializer_list<uint8_t> pins) {
		assert(max_strip_length < 1024 && "Max strip length is 1024, for reasonable refresh rate");
		assert(pins.size() <= 27 && "There are only 27 GPIO pins on rasperry pi zero. you can't possibly use more.");

		// Check the kernel module is loaded
		struct stat buffer;
		if (stat("/dev/ws2812_frame", &buffer) != 0) {
			std::cerr << "Looks like the ws2812 kernel module is not loaded. exiting!" << std::endl;
			exit(-1);
		}
		//std::cout << "Setting LED count to " << max_strip_length << std::endl;
		// Make sure CPU freq is constant.
		std::ofstream("/sys/devices/system/cpu/cpufreq/policy0/scaling_governor") << "performance" << std::endl;
		// Set LED count in the kernel
		std::ofstream("/sys/devices/virtual/ws2812/ws2812_frame/led_count") << max_strip_length << std::endl;
		
		pin_count = pins.size();
		int i = 0;
		uint32_t pins_map = 0;
		// Create strip to pin map
		for (auto pin : pins)
		{
			//std::cout << "Adding pin " << (int)pin << " at index " << i << std::endl;
			strip_to_pin[i++] = pin;
			pins_map |= (1 << pin);
		}
		// Set mask in kernel.
		std::ofstream("/sys/devices/virtual/ws2812/ws2812_frame/pin_mask") << "0x" << std::hex << pins_map << std::endl;

		led_count = max_strip_length;
		// Allocate buffers. 
		bits_data = new uint32_t[led_count * 24];
		pixels = new RGB_t[max_strip_length * pin_count];
	}

	~raspi_ws2812() {
		// release memory
		delete[] pixels;
		delete[] bits_data;
	}

	uint32_t max_strip_length() { return led_count;  }
	
	// Set color for pixel in strip.
	void SetPixel(uint8_t strip, uint32_t pixel, RGB_t color)
	{
		if (strip > pin_count || pixel > led_count) return;
		pixels[led_count * strip + pixel] = color;
	}
	
	// Get color for pixel in strip
	RGB_t GetPixel(uint8_t strip, uint32_t pixel)
	{
		if (strip > pin_count || pixel > led_count) RGB_t();
		return pixels[led_count * strip + pixel];
	}

	// Set all pixels to black
	void Clear() {
		for (size_t i = 0; i < pin_count * led_count; i++)
			pixels[i].raw = 0;
	}

	// Dim all pixels by factor
	void Dim(float factor) {
		for (size_t i = 0; i < pin_count * led_count; i++)
		{
			pixels[i].b = (uint8_t)(pixels[i].g * factor);
			pixels[i].r = (uint8_t)(pixels[i].g * factor);
			pixels[i].g = (uint8_t)(pixels[i].g * factor);
		}
	}

	//Send frame to kernel. This is where the magic happens
	void SendFrame() {
		static_assert(sizeof(RGB_t) == sizeof(uint32_t), "Expecting a 32bit RGB_t!");
		// Build the frame memory. 
		// bits-data is the frame going to the kernel. 
		// Each item is a mask of of the bits that are zero, and hence need to bu shut down early.
		// This logic was moved here to keep the kernel module simple. 
		for (uint32_t i = 0; i < led_count; i++) {
			for (int bit = 23; bit >= 0; bit--) {
				bits_data[24 * i + 23 - bit] = 0;
				for (unsigned int pin_index = 0; pin_index < pin_count; pin_index++) {
					auto as_unsigned = pixels[led_count * pin_index + i].raw;
					unsigned reverse_bit_data = ((~as_unsigned) & (1 << bit)) >> bit;
					bits_data[24 * i + 23 - bit] |= (reverse_bit_data << strip_to_pin[pin_index]);
				}
			}
		}
		
		// Send the frame to the kernel module
		int fd = open("/dev/ws2812_frame", O_WRONLY);
		if (fd == -1) {
			std::cerr << "failed opening device!" << std::endl;
			return;
		}
		ssize_t toWrite = led_count * 24 * sizeof(bits_data[0]);
		ssize_t size = write(fd, (const char*)bits_data, toWrite);
		if (toWrite != size) {
			std::cerr << "Failed writing enough data to frame. wanted: " << toWrite << " wrote: " << size << std::endl;
		}
		// This will display the frame
		close(fd);
	}

private:
	int8_t strip_to_pin[32];	// Map of strip number to GPIO pin, set in the constructor
	size_t pin_count;			// Count of pins.
	uint32_t led_count;			// Max number of LEDs per strip

	RGB_t* pixels;				// Pointer to pixels abstraction for the user
	uint32_t* bits_data;		// Memory for the built frame to be sent to the kernel

};
