/*
*  WS2812 on as many GPIO lines as you'd like kernel module
*
*  This kernel module takes which GPIO pins you'd like to use for WS2812 LEDs and the led count as parameters. 
*  Whenever the char device is closed it will freeze the kernel, send the data and resume the system.
*
*  Written by Uriel Guy, from http://www.urielguy.com or http://github.com/urielguy
*/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <asm/io.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/sysfs.h>


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Uriel Guy");    ///< The author -- visible when you use modinfo
MODULE_DESCRIPTION("A LKM for talking to as many WS2812 stripe as you want");  ///< The description -- see modinfo
MODULE_VERSION("0.1");

#define  DEVICE_NAME "ws2812_frame"
#define  CLASS_NAME  "ws2812"

// Couldn't find in any H file on the raspberry pi
#define GPIO_BASE (0x20000000 + 0x200000)
#define INTERRUPTS_BASE (0x20000000 + 0xB200)

static int max_leds = 100;				// Max LED strip length stored here
static uint32_t all_bits = 0;			// Bitmap of GPIO pins we care about.
static uint32_t* frameBuff = NULL;		// Pointer to kernel memory for frame
static uint8_t* frame_ptr = NULL;		// Pointer to current location while getting data from char device
static int is_open = false;				// Is char device currently open
static int    majorNumber;              // Stores the device number -- determined automatically
static struct class*  ws2812Class = NULL; // The device-driver class struct pointer
static struct device* ws2812Device = NULL; // The device-driver device struct pointer
static struct device_attribute pin_mask_attribute; // Attributes for pin mask 
static struct device_attribute led_count_attribute; // Attributes for strip length

// The prototype functions for the character driver -- must come before the struct definition
static int     dev_open(struct inode *, struct file *);
static int     dev_release(struct inode *, struct file *);
static ssize_t dev_write(struct file *, const char *, size_t, loff_t *);

/** @brief Devices are represented as file structure in the kernel. The file_operations structure from
*  /linux/fs.h lists the callback functions that you wish to associated with your file operations
*  using a C99 syntax structure. char devices usually implement open, read, write and release calls
*/
static struct file_operations fops =
{
	.open = dev_open,
	.read = NULL, // dev_read,
	.write = dev_write,
	.release = dev_release,
};


#pragma pack (push)
#pragma pack (0)

// GPIO Registers structure
struct GpioRegisters
{
	uint32_t GPFSEL[6];
	uint32_t Reserved1;
	uint32_t GPSET[2];
	uint32_t Reserved2;
	uint32_t GPCLR[2];
};

// Interrupts registers structure
struct InterruptsRegisters
{
	uint32_t IRQBasicPending;
	uint32_t IRQPending1;
	uint32_t IRQPending2;
	uint32_t FIQControl;
	uint32_t EnableIRQs1;
	uint32_t EnableIRQs2;
	uint32_t EnableBasicIRQs;
	uint32_t DisableIRQs1;
	uint32_t DisableIRQs2;
	uint32_t DisableBasicIRQs;
};
#pragma pack (pop)

// Sets the state of a GPIO pin using the registers
static void SetGPIOFunction(struct GpioRegisters* registers, int GPIO, int functionCode)
{
	int registerIndex = GPIO / 10;
	int bit = (GPIO % 10) * 3;

	unsigned oldValue = registers->GPFSEL[registerIndex];
	unsigned mask = 0b111 << bit;
//	printk("Changing function of GPIO%d from %x to %x\n",
		//GPIO,
		//(oldValue >> bit) & 0b111,
		//functionCode);

	registers->GPFSEL[registerIndex] =
		(oldValue & ~mask) | ((functionCode << bit) & mask);
}

// When file is opened check if not previously opened and reset the location to the head of the frame
static int dev_open(struct inode *inodep, struct file *filep) {
	if (is_open) return 1;
	is_open = 1;
	frame_ptr = (uint8_t*)frameBuff;
	return 0;
}

/*
* Store framne data to buffer
*/
static ssize_t dev_write(struct file *filep, const char *buffer, size_t len, loff_t *offset) {
	if (frame_ptr + len > (uint8_t*)(frameBuff + max_leds * 24)) {
		printk(KERN_WARNING "WS2812: Too much data for frame\n");
		return 0;
	}
	memcpy((void*)frame_ptr, buffer, len);
	frame_ptr += len;
	return len;
}


// Turn off optimization so the for delays work and to make sure this is consistent every time.
#pragma GCC push_options
#pragma GCC optimize ("O0")

// Device is reaesd. send the frame out
static int dev_release(struct inode *inodep, struct file *filep) {
	uint32_t* bit_data;
	uint32_t* end;
	int i;
	uint32_t irq1Val, irq2Val, basicIRQVal;
	uint32_t next_val;
	uint32_t counter;
	struct GpioRegisters *pGpioRegisters;
	struct InterruptsRegisters* pInterrupts;
	is_open = 0;

	if (max_leds == 0)
	{
		return 0;
	}

	// Map registers
	pGpioRegisters = (struct GpioRegisters *)ioremap(GPIO_BASE, sizeof(struct GpioRegisters));
	pInterrupts = (struct InterruptsRegisters*)ioremap(INTERRUPTS_BASE, sizeof(struct InterruptsRegisters));

	// Store interrupts register values so we can restore later
	irq1Val = readl(&pInterrupts->EnableIRQs1);
	irq2Val = readl(&pInterrupts->EnableIRQs2);
	basicIRQVal = readl(&pInterrupts->EnableBasicIRQs);

	// Stop all interrupts on the processor. This ractically stops the kernel. 
	writel(0xFFFFFFFF, &pInterrupts->DisableBasicIRQs);
	writel(0xFFFFFFFF, &pInterrupts->DisableIRQs2);
	writel(0xFFFFFFFF, &pInterrupts->DisableIRQs1);

	// get pointer to end of frame
	end = frameBuff + (24 * max_leds);

	// Prefetch and prime first 16 bytes
	for (i = 0; i < 16 ; i++) {
		__builtin_prefetch(frameBuff + i);
		writel(*(frameBuff + i), &pGpioRegisters->GPCLR[0]);
		writel(0, &pGpioRegisters->GPSET[0]);
	}

	// Clear all IO lines to 0.
	writel_relaxed(all_bits, &pGpioRegisters->GPCLR[0]);

	counter = 0;
	// Prefetch first value, just becaue better safe than sorry
	next_val = frameBuff[0];
	// Prime meory and IO system to prevent bad timings on first bit
	for (bit_data = frameBuff; bit_data < end; bit_data++) {
		// To make sure first bit is not too long, we act differently if counter == 1. 
		// If this is outside of the loop, bit 2 becomes problematic.
		// This loop sets all bits to 1, then clears all bits that need to be zero, and then all bits. 
		if (counter == 0) writel(0, &pGpioRegisters->GPSET[0]);
		if (counter == 0) writel(all_bits, &pGpioRegisters->GPCLR[0]);
		writel(all_bits, &pGpioRegisters->GPSET[0]);
		next_val = *bit_data & all_bits;
		for (i = ((counter == 0) ? 1 : 32); --i; );
		writel(next_val, &pGpioRegisters->GPCLR[0]);
		__builtin_prefetch(bit_data + 8);
		for (i = ((counter == 0) ? 5 : 55) ; --i; );
		writel(all_bits, &pGpioRegisters->GPCLR[0]);
		for (i = 28; --i; );
		counter++;
	}
	
	// Restore the kernel to normal activity
	writel(basicIRQVal, &pInterrupts->EnableBasicIRQs);
	writel(irq2Val, &pInterrupts->EnableIRQs2);
	writel(irq1Val, &pInterrupts->EnableIRQs1);
	
	// relese memory mapping to registers
	iounmap(pGpioRegisters);
	iounmap(pInterrupts);

	return 0;
}

#pragma GCC pop_options

ssize_t show_pin_mask(struct device * dev, struct device_attribute * attr, char * buf)
{
	return snprintf(buf, 12, "0x%08X\n", all_bits);
}

ssize_t show_led_count(struct device * dev, struct device_attribute * attr, char * buf)
{
	return snprintf(buf, 12, "%u\n", max_leds);
}

ssize_t store_pin_mask(struct device * dev, struct device_attribute * attr, const char * buf, size_t count)
{
	uint32_t new_bitmask, to_switch;
	struct GpioRegisters *pGpioRegisters;

	int i;

	printk(KERN_INFO "WS2812: store_pi_mask called\n");

	if (1 != sscanf(buf, "0x%x", &new_bitmask)) return count;

	printk(KERN_INFO "WS2812: new mask is 0x%08x\n", new_bitmask);

	pGpioRegisters = (struct GpioRegisters *)ioremap(GPIO_BASE, sizeof(struct GpioRegisters));

	// Sync GPIO pins state to the new mask. Don't touch other pins
	to_switch = all_bits & (~new_bitmask);
	for (i = 0; i < 32; i++)
	{
		if (to_switch & (1 << i)) {
			SetGPIOFunction(pGpioRegisters, i, 0);
		}
	}

	to_switch = (~all_bits) & (new_bitmask);
	for (i = 0; i < 32; i++)
	{
		if (to_switch & (1 << i)) {
			SetGPIOFunction(pGpioRegisters, i, 0);
			SetGPIOFunction(pGpioRegisters, i, 1);
		}
	}
	iounmap(pGpioRegisters);
	all_bits = new_bitmask;
	return count;
}

ssize_t store_led_count(struct device * dev, struct device_attribute * attr, const char * buf, size_t count)
{
	
	uint32_t new_val; 
	uint32_t* old_ptr;

	printk(KERN_INFO "WS2812: store_led_count called\n");
	
	if (1 != sscanf(buf, "%u", &new_val) || new_val > 1024)
	{
		printk(KERN_INFO "WS2812: invalid length value! (max size is 1024)\n");
		return count;
	}

	printk(KERN_INFO "WS2812: new length is %d\n", new_val);

	// Try and allocate frame meory. if fail, revert to old length.
	old_ptr = frameBuff;
	frameBuff = kmalloc(new_val * 24 * 4, GFP_KERNEL);
	if (frameBuff != NULL)
	{
		kfree((void*)old_ptr);
		max_leds = new_val;
		return count;
	}
	else {
		printk(KERN_INFO "WS2812: Allocation failed, reverting\n");
		frameBuff = old_ptr;
		return count;
	}
}
/** @brief The LKM initialization function
*  The static keyword restricts the visibility of the function to within this C file. The __init
*  macro means that for a built-in driver (not a LKM) the function is only used at initialization
*  time and that it can be discarded and its memory freed up after that point.
*  @return returns 0 if successful
*/
static int __init ws2812_init(void) {
	printk(KERN_INFO "WS2812: Initializing the WS2812 LKM\n");

	frameBuff = kmalloc(max_leds * 24 * 4, GFP_KERNEL);
	printk(KERN_INFO "WS2812: User-level access to CCR has been turned on.\n");

	// Try to dynamically allocate a major number for the device -- more difficult but worth it
	majorNumber = register_chrdev(0, DEVICE_NAME, &fops);
	
	if (majorNumber<0) {
		printk(KERN_ALERT "WS2812 failed to register a major number\n");
		return majorNumber;
	}
	printk(KERN_INFO "WS2812: registered correctly with major number %d\n", majorNumber);

	// Register the device class
	ws2812Class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(ws2812Class)) {                // Check for error and clean up if there is
		unregister_chrdev(majorNumber, DEVICE_NAME);
		printk(KERN_ALERT "WS2812: Failed to register device class\n");
		return PTR_ERR(ws2812Class);          // Correct way to return an error on a pointer
	}
	printk(KERN_INFO "WS2812: device class registered correctly\n");

	// Register the device driver
	ws2812Device = device_create(ws2812Class, NULL, MKDEV(majorNumber, 0), NULL, DEVICE_NAME);
	if (IS_ERR(ws2812Device)) {               // Clean up if there is an error
		class_destroy(ws2812Class);           // Repeated code but the alternative is goto statements
		unregister_chrdev(majorNumber, DEVICE_NAME);
		printk(KERN_ALERT "WS2812: Failed to create the device\n");
		return PTR_ERR(ws2812Device);
	}

	led_count_attribute.attr.name = "led_count";
	led_count_attribute.attr.mode = 0666;
	led_count_attribute.show = show_led_count;
	led_count_attribute.store = store_led_count;
	device_create_file(ws2812Device, &led_count_attribute);

	pin_mask_attribute.attr.name = "pin_mask";
	pin_mask_attribute.attr.mode = 0666;
	pin_mask_attribute.show = show_pin_mask;
	pin_mask_attribute.store = store_pin_mask;
	device_create_file(ws2812Device, &pin_mask_attribute);

	printk(KERN_INFO "WS2812: device class created correctly\n"); // Made it! device was initialized
	return 0;
}

/** @brief The LKM cleanup function
*  Similar to the initialization function, it is static. The __exit macro notifies that if this
*  code is used for a built-in driver (not a LKM) that this function is not required.
*/
static void __exit ws2812_exit(void) {
	printk(KERN_INFO "WS2812: Setting stop to true\n");
	device_remove_file(ws2812Device, &pin_mask_attribute);
	device_remove_file(ws2812Device, &led_count_attribute);
	printk(KERN_INFO "WS2812: Destroying device\n");
	device_destroy(ws2812Class, MKDEV(majorNumber, 0));     // remove the device
	printk(KERN_INFO "WS2812: Unregistering class\n");
	class_unregister(ws2812Class);                          // unregister the device class
	printk(KERN_INFO "WS2812: Destroying class\n");
	class_destroy(ws2812Class);                             // remove the device class
	printk(KERN_INFO "WS2812: Unregistering chardev\n");
	unregister_chrdev(majorNumber, DEVICE_NAME);             // unregister the major number
	printk(KERN_INFO "WS2812: Freeing frame\n");
	kfree((void*)frameBuff);
	printk(KERN_INFO "WS2812: Goodbye from the LKM!\n");
}

/** @brief A module must use the module_init() module_exit() macros from linux/init.h, which
*  identify the initialization function at insertion time and the cleanup function (as
*  listed above)
*/
module_init(ws2812_init);
module_exit(ws2812_exit);