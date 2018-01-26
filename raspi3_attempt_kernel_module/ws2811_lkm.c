#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/printk.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <asm/io.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/delay.h>


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Uriel Guy");    ///< The author -- visible when you use modinfo
MODULE_DESCRIPTION("A LKM for talking to as many WS2811 stripe as you want");  ///< The description -- see modinfo
MODULE_VERSION("0.1");

#define  DEVICE_NAME "ws2811_frame"    ///< The device will appear at /dev/ebbchar using this value
#define  CLASS_NAME  "ws2811"        ///< The device class -- this is a character device driver

// Couldn't find in any H file on the raspberry pi
#define GPIO_BASE (0x3F000000 + 0x200000)
#define CORE3_TIMER_IRQCNTL  0x4000004C
#define CORE3_MBOX_IRQCNTL   0x4000005C


#define MAX_LEDS 100
#define FRAME_LENGTH (24 * MAX_LEDS)
#define FRAME_SIZE (FRAME_LENGTH * sizeof(uint32_t))

static struct task_struct* m_thread = NULL;
static volatile int stop = false;
static volatile uint32_t frame = 0;
static volatile uint32_t* frameBuff = NULL;
static volatile uint8_t* frame_ptr = NULL;
static int is_open = false;
static int    majorNumber;                  ///< Stores the device number -- determined automatically
static struct class*  ws2811Class = NULL; ///< The device-driver class struct pointer
static struct device* ws2811Device = NULL; ///< The device-driver device struct pointer

// The prototype functions for the character driver -- must come before the struct definition
static int     dev_open(struct inode *, struct file *);
static int     dev_release(struct inode *, struct file *);
//static ssize_t dev_read(struct file *, char *, size_t, loff_t *);
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

/** @brief The device open function that is called each time the device is opened
*  This will only increment the numberOpens counter in this case.
*  @param inodep A pointer to an inode object (defined in linux/fs.h)
*  @param filep A pointer to a file object (defined in linux/fs.h)
*/
static int dev_open(struct inode *inodep, struct file *filep) {
	if (is_open) return 1;
	is_open = 1;
	frame_ptr = (uint8_t*)frameBuff;
	printk(KERN_INFO "WS2811: Frame open\n");
	return 0;
}

/** @brief This function is called whenever the device is being written to from user space i.e.
*  data is sent to the device from the user. The data is copied to the message[] array in this
*  LKM using the sprintf() function along with the length of the string.
*  @param filep A pointer to a file object
*  @param buffer The buffer to that contains the string to write to the device
*  @param len The length of the array of data that is being passed in the const char buffer
*  @param offset The offset if required
*/
static ssize_t dev_write(struct file *filep, const char *buffer, size_t len, loff_t *offset) {
	if (frame_ptr + len > (uint8_t*)(frameBuff + FRAME_LENGTH)) {
		printk(KERN_WARNING "WS2811: Too much data for frame\n");
	}
	printk(KERN_INFO "WS2811: Received %zu characters from the user\n", len);
	memcpy((void*)frame_ptr, buffer, len);
	frame_ptr += len;
	return len;
}

/** @brief The device release function that is called whenever the device is closed/released by
*  the userspace program
*  @param inodep A pointer to an inode object (defined in linux/fs.h)
*  @param filep A pointer to a file object (defined in linux/fs.h)
*/
static int dev_release(struct inode *inodep, struct file *filep) {
	printk(KERN_INFO "WS2811: Device successfully closed with %d samples. Sample 1: %x\n", (uint32_t*)frame_ptr - frameBuff, frameBuff[0]);
	is_open = 0;
	frame++;
	return 0;
}

struct GpioRegisters
{
	uint32_t GPFSEL[6];
	uint32_t Reserved1;
	uint32_t GPSET[2];
	uint32_t Reserved2;
	uint32_t GPCLR[2];
};


static inline unsigned ccnt_read(void)
{
	unsigned cc;
	asm volatile ("mrc p15, 0, %0, c15, c12, 1" : "=r" (cc));
	return cc;
}

static inline unsigned ccnt_read2(void)
{
	unsigned cc;
	__asm__ __volatile__("mrc p15, 0, %0, c9, c13, 0" : "=r"(cc));
	return cc;
}

static void SetGPIOFunction(struct GpioRegisters* registers, int GPIO, int functionCode)
{
	int registerIndex = GPIO / 10;
	int bit = (GPIO % 10) * 3;

	unsigned oldValue = registers->GPFSEL[registerIndex];
	unsigned mask = 0b111 << bit;
	printk("Changing function of GPIO%d from %x to %x\n",
		GPIO,
		(oldValue >> bit) & 0b111,
		functionCode);

	registers->GPFSEL[registerIndex] =
		(oldValue & ~mask) | ((functionCode << bit) & mask);
}

//TODO: Read from chardev/configure
static uint32_t all_bits = (1 << 5);
static int ThreadFunc(void* param) {
	uint32_t last_frame;
	uint32_t* my_frame;
	uint32_t* end_frame;
	uint32_t* bit_data;
	uint32_t timestamp;
	int i;
	uint32_t timerVal;
	uint32_t mailboxVal;
	uint32_t next_val;
	struct GpioRegisters *pGpioRegisters;
	uint32_t* pTimersInterrupts;
	uint32_t* pMailboxInterrupts;


	printk(KERN_INFO "WS2811_THREAD: Allocating frame\n");

	pGpioRegisters = (struct GpioRegisters *)ioremap(GPIO_BASE, sizeof(struct GpioRegisters));
	pMailboxInterrupts = (uint32_t*)ioremap(CORE3_MBOX_IRQCNTL, sizeof(uint32_t));
	pTimersInterrupts = (uint32_t*)ioremap(CORE3_TIMER_IRQCNTL, sizeof(uint32_t));

	last_frame = 0;
	my_frame = kmalloc(FRAME_SIZE, GFP_KERNEL);
	end_frame = my_frame + FRAME_LENGTH;
	if (my_frame == NULL)
	{
		printk(KERN_INFO "WS2811_THREAD: Failed allocating ws2811 frame memory!\n");
		stop = true;
		do_exit(-1);
		return -1;
	}
	printk(KERN_INFO "WS2811_THREAD: Entering loop\n");
	SetGPIOFunction(pGpioRegisters, 5, 0);
	SetGPIOFunction(pGpioRegisters, 5, 1);

	while (!stop)
	{
	//	printk(KERN_INFO "WS2811_THREAD: Waiting. last_frame = %u\n", last_frame);
		while (frame == last_frame) yield();
//		set_current_state(TASK_UNINTERRUPTIBLE);
		timerVal = (uint32_t)readl(pTimersInterrupts);
		mailboxVal = (uint32_t)readl(pMailboxInterrupts);
		writel(timerVal & 0xFFFFFF00, pTimersInterrupts);
		writel(mailboxVal & 0xFFFFFF00, pMailboxInterrupts);
	//	printk(KERN_INFO "WS2811_THREAD: Got frame. copying locally.%u\n", last_frame);
		last_frame = frame;
		__asm__ __volatile__("mcr p15, 0, %0, c9, c12, 2" :: "r"(1 << 31)); /* stop the cc */
		__asm__ __volatile__("mcr p15, 0, %0, c9, c12, 0" :: "r"(5));     /* initialize */
		__asm__ __volatile__("mcr p15, 0, %0, c9, c12, 1" :: "r"(1 << 31)); /* start the cc */
		memcpy(my_frame, (void *)frameBuff, FRAME_SIZE);
		//printk(KERN_INFO "WS2811_THREAD: Sending out frame %u. tickcknt: %u bit[0]: %x\n", last_frame, ccnt_read2(), my_frame[0]);
		for (i = 0; i < FRAME_LENGTH; i++) __builtin_prefetch(my_frame + i);
		writel_relaxed(all_bits, &pGpioRegisters->GPCLR[0]);
		timestamp = ccnt_read2();
		while (ccnt_read2() - timestamp < 1000);

		for (bit_data = my_frame; bit_data != end_frame; bit_data++) {
			writel_relaxed(all_bits, &pGpioRegisters->GPSET[0]);
			timestamp = ccnt_read2();
			next_val = *bit_data & all_bits;
			while (ccnt_read2() - timestamp < 400);
			writel_relaxed(next_val, &pGpioRegisters->GPCLR[0]);
			__builtin_prefetch(bit_data + 8);
			while (ccnt_read2() - timestamp < 1050);
			writel_relaxed(all_bits, &pGpioRegisters->GPCLR[0]);
			while (ccnt_read2() - timestamp < 1490);

		}
		writel(timerVal, pTimersInterrupts);
		writel(mailboxVal, pMailboxInterrupts);
//		set_current_state(TASK_RUNNING);
	}
	iounmap(pGpioRegisters);
	iounmap(pMailboxInterrupts);
	iounmap(pTimersInterrupts);
	printk(KERN_INFO "WS2811_THREAD: Loop exited\n");
	kfree(my_frame);
	printk(KERN_INFO "WS2811_THREAD: Bye bye\n");
	do_exit(0);
	return 0;
}

/** @brief The LKM initialization function
*  The static keyword restricts the visibility of the function to within this C file. The __init
*  macro means that for a built-in driver (not a LKM) the function is only used at initialization
*  time and that it can be discarded and its memory freed up after that point.
*  @return returns 0 if successful
*/
static int __init ws2811_init(void) {
	uint32_t now; 
	printk(KERN_INFO "WS2811: Initializing the WS2811 LKM\n");

	__asm__ __volatile__("mcr p15, 0, %0, c9, c12, 2" :: "r"(1 << 31)); /* stop the cc */
	__asm__ __volatile__("mcr p15, 0, %0, c9, c12, 0" :: "r"(5));     /* initialize */
	__asm__ __volatile__("mcr p15, 0, %0, c9, c12, 1" :: "r"(1 << 31)); /* start the cc */
	frameBuff = kmalloc(FRAME_SIZE, GFP_KERNEL);
	printk(KERN_INFO "WS2811: User-level access to CCR has been turned on.\n");
	now = ccnt_read2();

	printk(KERN_INFO "WS2811: tickCount: %u.\n", now);


	printk(KERN_INFO "Got GPIO registers. stariting thread\n");

	m_thread = kthread_create(ThreadFunc, NULL, "WS2812_GPIO");
	
	if (IS_ERR(m_thread)) {
		printk("WS2811: Unable to start kernel thread.\n");
		return -2;
	}
	else {
		printk(KERN_INFO "WS2811: Thread created Binding!\n");
		kthread_bind(m_thread, 3);
		printk(KERN_INFO "WS2811: Thread created on core 3. Waking up!\n");
		wake_up_process(m_thread);
	}
	printk(KERN_INFO "WS2811: Thread bound. registering device!\n");


	// Try to dynamically allocate a major number for the device -- more difficult but worth it
	majorNumber = register_chrdev(0, DEVICE_NAME, &fops);
	
	if (majorNumber<0) {
		printk(KERN_ALERT "WS2811 failed to register a major number\n");
		return majorNumber;
	}
	printk(KERN_INFO "WS2811: registered correctly with major number %d\n", majorNumber);

	// Register the device class
	ws2811Class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(ws2811Class)) {                // Check for error and clean up if there is
		unregister_chrdev(majorNumber, DEVICE_NAME);
		printk(KERN_ALERT "WS2811: Failed to register device class\n");
		return PTR_ERR(ws2811Class);          // Correct way to return an error on a pointer
	}
	printk(KERN_INFO "WS2811: device class registered correctly\n");

	// Register the device driver
	ws2811Device = device_create(ws2811Class, NULL, MKDEV(majorNumber, 0), NULL, DEVICE_NAME);
	if (IS_ERR(ws2811Device)) {               // Clean up if there is an error
		class_destroy(ws2811Class);           // Repeated code but the alternative is goto statements
		unregister_chrdev(majorNumber, DEVICE_NAME);
		printk(KERN_ALERT "WS2811: Failed to create the device\n");
		return PTR_ERR(ws2811Device);
	}
	printk(KERN_INFO "WS2811: device class created correctly\n"); // Made it! device was initialized
	return 0;
}

/** @brief The LKM cleanup function
*  Similar to the initialization function, it is static. The __exit macro notifies that if this
*  code is used for a built-in driver (not a LKM) that this function is not required.
*/
static void __exit ws2811_exit(void) {
	printk(KERN_INFO "WS2811: Setting stop to true\n");
	stop = true;
	printk(KERN_INFO "WS2811: Destroying device\n");
	device_destroy(ws2811Class, MKDEV(majorNumber, 0));     // remove the device
	printk(KERN_INFO "WS2811: Unregistering class\n");
	class_unregister(ws2811Class);                          // unregister the device class
	printk(KERN_INFO "WS2811: Destroying class\n");
	class_destroy(ws2811Class);                             // remove the device class
	printk(KERN_INFO "WS2811: Unregistering chardev\n");
	unregister_chrdev(majorNumber, DEVICE_NAME);             // unregister the major number
	printk(KERN_INFO "WS2811: Sleeping 2 secs\n");
	msleep(500);
	//printk(KERN_INFO "WS2811: Stopping thread\n");
	//kthread_stop(m_thread);
	printk(KERN_INFO "WS2811: Unmapping GPIO\n");
	printk(KERN_INFO "WS2811: Freeing frame\n");
	kfree((void*)frameBuff);
	printk(KERN_INFO "WS2811: Goodbye from the LKM!\n");
}

/** @brief A module must use the module_init() module_exit() macros from linux/init.h, which
*  identify the initialization function at insertion time and the cleanup function (as
*  listed above)
*/
module_init(ws2811_init);
module_exit(ws2811_exit);