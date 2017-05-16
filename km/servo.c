#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h> /* printk() */
#include <linux/slab.h> /* kmalloc() */
#include <linux/fs.h> /* everything... */
#include <linux/errno.h> /* error codes */
#include <linux/types.h> /* size_t */
#include <linux/fcntl.h> /* O_ACCMODE */
#include <linux/jiffies.h> /* jiffies */
#include <asm/system.h> /* cli(), *_flags */
#include <asm/uaccess.h> /* copy_from/to_user */
#include <linux/interrupt.h>
#include <asm/arch/gpio.h>
#include <asm/hardware.h>
#include <asm/arch/pxa-regs.h>
#include <linux/clocksource.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <asm/mach/time.h>

MODULE_LICENSE("Dual BSD/GPL");

/** Define servo file ops */
/* Define servo file operator function headers (open, release, write) */
static int servo_open(struct inode *inode, struct file *filp);
static int servo_release(struct inode *inode, struct file *filp);
static ssize_t servo_write(struct file *filp, const char *buf, size_t count, loff_t *f_pos);

/* servo file access structure */
struct file_operations servo_fops = {
    write: servo_write,
    open: servo_open,
    release: servo_release
};

/* Prototypes */
static int servo_init(void);
static void servo_exit(void);

module_init(servo_init);
module_exit(servo_exit);

uint32_t servo_val0 = 0;
uint32_t servo_val1 = 0;
uint32_t servo_cycles = 0;

static irqreturn_t servo_timer(int irq, void *dev_id) {
    int next_match;
    do {
        OSSR = OSSR_M1;  /* Clear match on timer 1 */
        next_match = (OSMR1 += 10000);
        servo_cycles += 1;
    } while( (signed long)(next_match - OSCR) <= 8 );

    // Send servo_valN to the appropriate PWM channel when the time is right.
    PWM_PWDUTY0 = ((servo_cycles & 0x3) == 0x3)?servo_val0:0;
    PWM_PWDUTY1 = ((servo_cycles & 0x3) == 0x3)?servo_val1:0;
    return IRQ_HANDLED;
}

static int servo_init(void) {
    /** Register character driver for servo **/
    int ret;
    ret = register_chrdev(61, "servo", &servo_fops);
    if (ret < 0) {
        printk(KERN_ALERT "servo did not register\n");
        return ret;
    }
    pxa_gpio_mode(GPIO16_PWM0_MD); // setup GPIO16 as PWM0
    pxa_set_cken(CKEN0_PWM0,1); //Enable the PWM0 Clock
    servo_val0 = 0;
    PWM_PWDUTY0 = servo_val0;
    PWM_CTRL0 = 0x3F;
    PWM_PERVAL0 = 0x3FF;

    pxa_gpio_mode(GPIO17_PWM1_MD); // setup GPIO17 as PWM1
    pxa_set_cken(CKEN1_PWM1,1); //Enable the PWM1 Clock
    servo_val1 = 0;
    PWM_PWDUTY1 = servo_val1;
    PWM_CTRL1 = 0x3F;
    PWM_PERVAL1 = 0x3FF;
    
    // Initialize OS TIMER
    if (request_irq(IRQ_OST1, &servo_timer, IRQF_TIMER | IRQF_DISABLED, "", NULL)) {
        servo_exit();
	return 0;
    }

    OIER |= OIER_E1; /* enable match on timer match 1 to cause interrupts */
    // Install OS Timer Match 1 interrupt
    OSMR1 = OSCR + 10000; /* set initial match */
    printk(KERN_ALERT "Installed Servo\n");
    return 0;
}


static void servo_exit(void) {	
    unregister_chrdev(61, "servo");

    // Disable and release PWM0 and PWM1 clocks
    pxa_set_cken(CKEN0_PWM0,0); 
    pxa_set_cken(CKEN1_PWM1,0); 

    OIER = OIER & (~OIER_E1); /* disable match on timer match 1 to cause interrupts */
    free_irq(IRQ_OST1, NULL);
    printk(KERN_ALERT "Removing servo module\n");
}

// carried out when /dev/servo is opened
static int servo_open(struct inode *inode, struct file *filp)
{
    /* Success */
    return 0;
}

// carried out when /dev/servo is closed
static int servo_release(struct inode *inode, struct file *filp)
{
    /* Success */
    return 0;
}

// when values are written into /dev/servo file
static ssize_t servo_write(struct file *filp, const char *buf, size_t count,
                            loff_t *f_pos)
{
    char buffer[8];    
    uint32_t res;
     // Copy write buffer into kernel memory
     if (copy_from_user(buffer, buf, 8)) { 
         return -EFAULT; 
     }

         if (buffer[0] == 't') { // for tilt servo
         res = simple_strtol((buffer+1), NULL, 10); // goes from 120 - 500
         servo_val0 = res;
         
     } else if (buffer[0] =='p') { // for pan servo
         res = simple_strtol((buffer+1), NULL, 10); // goes from 120 - 500
         servo_val1 = res;
         
     } else {
         return -EFAULT;
     }

    return count;
}



