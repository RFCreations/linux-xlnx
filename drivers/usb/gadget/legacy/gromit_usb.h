/*
 * NB Slowly morphing into Gromit driver
 */

#ifndef __G_GROMIT_USB_H
#define __G_GROMIT_USB_H

#include <linux/usb/composite.h>

/* Define debugging for use during our driver bringup */
//#define CONFIG_GROMIT_USB_DEBUG
#ifdef CONFIG_GROMIT_USB_DEBUG
#define PDEBUG(fmt, args...) printk(KERN_INFO fmt, ## args)
#else
#define PDEBUG(fmt, args...) 
#endif

#endif // __G_GROMIT_USB_H
