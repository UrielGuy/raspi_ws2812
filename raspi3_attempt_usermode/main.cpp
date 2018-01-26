#include <thread>
#include <iostream>
#include <fstream>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <atomic>
#include <cassert>
#include <cstring>
#include <csignal>

#include "mailbox.h"
#include "InterruptRegisters.h"

#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define GPIO_SET *(gpio+7)  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR *(gpio+10) // clears bits which are 1 ignores bits which are 0

volatile unsigned* gpio;

//const std::array<int, 9> WS_GPIO = { 5 ,6, 13, 19, 26, 21, 20, 16, 12 };
const std::array<int, 1> WS_GPIO = { 5 };
constexpr int MAX_STRIP_LENGTH = 20;

static_assert(MAX_STRIP_LENGTH <= 150, "We limiting the size so that all the data can fit in the L1 data (16KB for raspi3) buffer.");

// We need one unsigned (size of GPIO register) for each bit int the protocol
std::array<unsigned, MAX_STRIP_LENGTH * 3 * 8 > bitsData1;

volatile size_t frameNumber{ 0 };

void InitAndVerifyCore3() {
    // Set affinty to core 3.
    cpu_set_t cpuset;
    auto thread = pthread_self();

    CPU_ZERO(&cpuset);
    CPU_SET(3, &cpuset);

    auto s = pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset);
    if (s != 0)
    {
        std::cerr << "Failed to set processor affinty to core 3" << std::endl;
        exit(-1);
    }

    std::ifstream t1("/sys/devices/system/cpu/isolated");
    std::string isol_str((std::istreambuf_iterator<char>(t1)),
        std::istreambuf_iterator<char>());

    if (isol_str != "3\n")
    {
        std::cerr << "Core 3 is not isolated" << std::endl;
        exit(-1);
    }

    {
        std::ofstream out_file("/sys/devices/system/cpu/cpu3/cpufreq/scaling_min_freq");
        out_file << "600000\n";
        out_file.flush();
    }
    {
        std::ofstream out_file("/sys/devices/system/cpu/cpu3/cpufreq/scaling_max_freq");
        out_file << "600000\n";
        out_file.flush();
    }
    {
        std::ofstream out_file("/sys/devices/system/cpu/cpu3/cpufreq/scaling_governor");
        out_file << "pwersave\n";
        out_file.flush();
    }
    usleep(100000);
    std::ifstream t2("/sys/devices/system/cpu/cpu3/cpufreq/cpuinfo_cur_freq");
    std::string cur_freq((std::istreambuf_iterator<char>(t2)),
        std::istreambuf_iterator<char>());

    std::ifstream t3("/sys/devices/system/cpu/cpu3/cpufreq/cpuinfo_min_freq");
    std::string min_freq((std::istreambuf_iterator<char>(t3)),
        std::istreambuf_iterator<char>());

    std::ifstream t4("/sys/devices/system/cpu/cpu3/cpufreq/cpuinfo_max_freq");
    std::string max_freq((std::istreambuf_iterator<char>(t4)),
        std::istreambuf_iterator<char>());

    if ("600000\n" != cur_freq) {
        std::cerr << "Core 3 clock speed is not stabilized" << std::endl;
        exit(-1);
    }

}

void InitGPIO() {
    const auto BLOCK_SIZE(4 * 1024);

    auto mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_fd < 0) {
        printf("can't open /dev/mem \n");
        exit(-1);
    }

    std::ifstream t("/proc/cpuinfo");
    std::string str((std::istreambuf_iterator<char>(t)),
        std::istreambuf_iterator<char>());

    __off_t BCM2708_PERI_BASE = 0;
    BCM2708_PERI_BASE = 0x3F000000;
    std::cout << "Using raspi3 registers\n";
    auto GPIO_BASE = (BCM2708_PERI_BASE + 0x200000); /* GPIO controller */

                                                     /* mmap GPIO */
    auto gpio_map = mmap(
        NULL,             //Any adddress in our space will do
        BLOCK_SIZE,       //Map length
        PROT_READ | PROT_WRITE,// Enable reading & writting to mapped memory
        MAP_SHARED,       //Shared with other processes
        mem_fd,           //File to map
        GPIO_BASE         //Offset to GPIO peripheral
    );

    close(mem_fd); //No need to keep mem_fd open after mmap

    if (gpio_map == MAP_FAILED) {
        printf("mmap error %d\n", (int)gpio_map);//errno also set!
        exit(-1);
    }

    // Always use volatile pointer!
    gpio = (volatile unsigned *)gpio_map;


    for (auto pin : WS_GPIO) {
        INP_GPIO(pin); // must use INP_GPIO before we can use OUT_GPIO
        OUT_GPIO(pin);
    }
}

volatile unsigned* CORE3_TIMER_IRQCNTL_REG = nullptr;
volatile unsigned* CORE3_MBOX_IRQCNTL_REG = nullptr;

void CreateCore3InterruptsRegs() {
    const auto BLOCK_SIZE(4 * 1024);

    auto mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_fd < 0) {
        printf("can't open /dev/mem \n");
        exit(-1);
    }
#define IRQ_REG_BASE 0x40000000

    auto irq_map = mmap(
        NULL,             //Any adddress in our space will do
        BLOCK_SIZE,       //Map length
        PROT_READ | PROT_WRITE,// Enable reading & writting to mapped memory
        MAP_SHARED,       //Shared with other processes
        mem_fd,           //File to map
        IRQ_REG_BASE         //Offset to IRQ registeras
    );

    close(mem_fd); //No need to keep mem_fd open after mmap

    if (irq_map == MAP_FAILED) {
        printf("mmap error %d\n", (int)irq_map);//errno also set!
        exit(-1);
    }

    // Always use volatile pointer!
    auto irq  = (volatile unsigned *)irq_map;
#define IRQ_REG(x) ((volatile unsigned&)(*(((volatile uint8_t*)irq) + (x - 0x40000000))))
    
	CORE3_TIMER_IRQCNTL_REG = &IRQ_REG(CORE3_TIMER_IRQCNTL);
	CORE3_MBOX_IRQCNTL_REG = &IRQ_REG(CORE3_MBOX_IRQCNTL);
	//IRQ_REG(CORE3_MBOX_IRQCNTL) &= 0xFFFFFF00;

#undef IRQ_REG

}

#pragma GCC push_options
#pragma GCC optimize ("O0")

volatile bool stop{ false };
void WS2811Thread() {
    CreateCore3InterruptsRegs();
	*CORE3_MBOX_IRQCNTL_REG |= 0x01;
    *CORE3_TIMER_IRQCNTL_REG |= 0x03;

    InitAndVerifyCore3();
    InitGPIO();
    
    uint32_t localFrame[24 * MAX_STRIP_LENGTH + 24];
        
    register unsigned all_pins = 0;
    for (auto pin : WS_GPIO) all_pins |= (1 << pin);

    *CORE3_TIMER_IRQCNTL_REG &= ~0xFF;
	*CORE3_MBOX_IRQCNTL_REG &= ~0xFF;

    auto lastFrame = frameNumber;
    int delay;
    while (!stop) {
        while (!stop  && frameNumber == lastFrame);
        if (stop) break;
        auto data = bitsData1.data();

        auto size = bitsData1.size() * sizeof(bitsData1[0]);
        memcpy(localFrame, data, size);
        uint32_t* end = localFrame + bitsData1.size();
        lastFrame = frameNumber;
        uint32_t regValue = all_pins;
        for (int i = std::min((size_t)150 * 24, bitsData1.size()); i >= 0; i--) __builtin_prefetch(localFrame + i);
        for (uint32_t* bit = localFrame; bit != end; bit++) {
            GPIO_SET = all_pins; // Set all pins to 1
  //          regValue = *bit;
            for (int i = 0; i < 23; i++);
            GPIO_CLR = regValue;  // Set proper bits to 0
//            __builtin_prefetch(localFrame + 16);
            for (int i = 0; i < 50; i++);
            GPIO_CLR = all_pins; // Clear everything to zero
            for (int i = 0; i < 23; i++);
        }
        GPIO_CLR = all_pins; // Clear everything to zero
        for (delay = 210 * 30; delay; delay--); // New frame.

    }

    *CORE3_TIMER_IRQCNTL_REG |= 0x03;
	*CORE3_MBOX_IRQCNTL_REG |= 0x01;

}
#pragma GCC pop_options

#pragma pack (push)
#pragma pack (0)
struct RGB_t {
    union {
        struct {
            uint8_t b, r, g;
            uint8_t padding;
        };
        uint32_t raw;
    };
};
#pragma pack (pop)

RGB_t Pixels[WS_GPIO.size()][MAX_STRIP_LENGTH];

void SendFrame() {
    static_assert(sizeof(RGB_t) == sizeof(uint32_t), "Expecting a 32bit RGB_t!");
    for (int i = 0; i < MAX_STRIP_LENGTH; i++) {
        for (int bit = 23; bit >= 0; bit--) {
            bitsData1[24 * i + 23 -bit] = 0;
            for (unsigned int pin_index = 0; pin_index < WS_GPIO.size(); pin_index++) {
                auto as_unsigned = Pixels[pin_index][i].raw;
                unsigned reverse_bit_data = ((~as_unsigned) & (1 << bit)) >> bit;
                bitsData1[24 * i + 23 - bit] |= (reverse_bit_data << WS_GPIO[pin_index]);
//                std::cout << "Added " << std::hex << (reverse_bit_data << WS_GPIO[pin_index]) << std::dec << " to " << (24 * i + 23 - bit) << std::endl;
            }
        }
    }
//    for (auto a : bitsData1) std::cout << std::hex << a << std::endl;
    frameNumber++;
}
void HandleSigint(int) { std::cout << "stopping!" << std::endl; *CORE3_TIMER_IRQCNTL_REG |= 0x03; ;  stop = true; }
int main(void)
{
    signal(SIGINT, HandleSigint);
    std::thread ws2811_thread(WS2811Thread);
    memset(Pixels, 0, sizeof(Pixels[0]) * WS_GPIO.size() * MAX_STRIP_LENGTH);

    unsigned counter = 0;
    while (!stop) {
        ++counter;
        for (unsigned i = 0; i < WS_GPIO.size() ; i++) for (unsigned j = 0; j < MAX_STRIP_LENGTH; j++) {
            if (counter % 3 == j % 3)
            {
                Pixels[i][j].g = 0x55;
                Pixels[i][j].r = 0x55;
                Pixels[i][j].b = 0x55;
            }
            else {
                Pixels[i][j].g = 0x55;
                Pixels[i][j].r = 0x55;
                Pixels[i][j].b = 0x55;
            }
        }
        SendFrame();
        usleep(250000);
    }
    std::cout << "Waiting for thread to exit" << std::endl;
    ws2811_thread.join();

}