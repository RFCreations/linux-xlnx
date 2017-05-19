/*
 * gromit_usb.c -- Gadget Driver for Gromit
 *
 * Copyright (C) 2003-2008 David Brownell
 * Copyright (C) 2008 by Nokia Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */


/*
 * Gromit needs three bulk endpoints. These
 * are mapped to device files as follows:
 *
 * gromit_usb_data: handles IN bulk endpoint. No OUT data handled by this
 * device file (i.e. from the device's perspective, it only writes data to
 * the host, it doesn't read data.
 *
 * gromit_usb_ctrl: handles both IN and OUT bulk endpoint data - commands,
 * firmware upgrades, etc. So the device file both reads and writes data.
 */

/*
 * driver assumes self-powered hardware, and
 * has no way for users to trigger remote wakeup.
 */

#define VERBOSE_DEBUG

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/device.h>

#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/usb/composite.h>
#include <linux/delay.h>

#include "u_os_desc.h"

#include "gromit_usb.h"


/*-------------------------------------------------------------------------*/

MODULE_LICENSE("GPL");
MODULE_AUTHOR("RF Creations Ltd");
MODULE_DESCRIPTION("gromit_usb - USB gadget driver for Gromit");

/*-------------------------------------------------------------------------*/
#define DRIVER_NAME "gromit_usb"
#define DRIVER_VERSION  "2015-05-06"
#define DRIVER_VENDOR_NUM_DEF 0x2BBD /* RF Creations */
#define DRIVER_PRODUCT_NUM_DEF 0xF020
static ushort DRIVER_VENDOR_NUM __initdata = 0x2BBD; /* RF Creations */
static ushort DRIVER_PRODUCT_NUM __initdata = 0xF020;

module_param( DRIVER_VENDOR_NUM , ushort , S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP );
module_param( DRIVER_PRODUCT_NUM , ushort , S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP );

/*-------------------------------------------------------------------------*/
// Number of requests to allocate up front per endpoint
#define NUM_CTRL_REQS  1
#define NUM_DATA_REQS  64 //(65535UL)
#define REQ_LEN   4096
#define PKT_LEN   512L  // MUST be power of two!

/*-------------------------------------------------------------------------*/
#define FLUSH_TIMEOUT  (1250)  // 1.25s
#define CLOSE_TIMEOUT  (250)   // 250ms

// Linked-list entry for a block of memory
struct ep_req
{
   struct list_head node;
   struct usb_request *req;
   ssize_t offset;
};

//OS Descriptors
static const char os_string_desc[] = "MSFT100";
static const unsigned char vendor_code = 0xBD;

static char guid_reg_name[] = "DeviceInterfaceGUID";
static char guid[] = "{ef4569de-1eea-4687-86ec-dc110f910893}";

struct usb_os_desc_ext_prop gromit_os_desc_prop_guid = {
	.type = USB_EXT_PROP_UNICODE,
	.name_len = 2*sizeof(guid_reg_name),
	.name = guid_reg_name,
	.data_len = 2*sizeof(guid),
	.data = guid,
};

static char compatible_id[16] = "WINUSB\0\0\0\0\0\0\0\0\0";
static struct usb_os_desc gromit_os_desc = {
		.ext_compat_id = compatible_id,
		.ext_prop_count = 1,
		.ext_prop_len = 14 + 2*sizeof(guid_reg_name) + 2*sizeof(guid),
};

static struct usb_os_desc_table gromit_os_desc_table = {
		.if_id = 0,
		.os_desc = &gromit_os_desc,
};

// Private data for endpoints - endpoint plus linked list of usb_request
// structures.
//
// 'pending_req' is for requests queued in the USB controller
// 'free_req' is for requests that are not queued, and not waiting for
// user collection
// 'collect_req' is for OUT requests waiting for user collection
// 'user_req' is for OUT requests waiting to be processed in fop_read()
//
// Ideally all requests for 'OUT' transactions will be either be queued
// in 'pending' or waiting for collection in 'collect'. However, when the
// device is unplugged requests cannot be queued, so there can be entries
// in 'free' as well.
//
// When fop_read() is called, it transfers all entries in 'collect' into
// 'user', while holding a spinlock. Then it releases the spinlock and
// processes the entries in 'user'. This avoids problems with the callback 
// modifying 'collect' while fop_read() is running.
//
// For IN transactions, requests will all be 'free' until the 'write' function
// is called when requests will be pushed onto 'pending' to send the data
// over the bus. In this case 'collect' is unused.
struct gromit_ep_data
{
   struct usb_ep *ep;            // NULL if not used
   struct list_head pending_req; // Reqs pending transfer over USB
   struct list_head free_req;    // Reqs not in use
   struct list_head collect_req; // Reqs waiting collection
   struct list_head user_req;    // Reqs being handled by fop_read()
   bool collect_data;            // Set to TRUE when there's OUT data available
   unsigned int free_reqs;       // Current no. of reqs free
   unsigned int min_free_reqs;   // Min no. of reqs free
   unsigned int max_free_reqs;   // Max no. of reqs free
};

// Private data for device file handling
struct gromit_devfile
{
   struct device *dev;
   struct cdev cdev;
   atomic_t in_use;      // Only allow one user access at a time
   wait_queue_head_t wq; // Wait queue for blocking access 
   struct semaphore sem; // Exclusion semaphore
   spinlock_t lock;      // Spinlock for accessing user_data
   struct gromit_ep_data in_ep; // IN endpoint for this device
   struct gromit_ep_data out_ep; // OUT endpoint for this device
};

struct gromit_private_data
{
   struct usb_function function;
   struct class *class; /* for CHRDEV */
   struct gromit_devfile data_dev; // Device for data transfer
   struct gromit_devfile ctrl_dev; // Device for control messages 
   dev_t devnum;            // Allocated base device number (for both devices)
};

// singeton instance of private data
static struct gromit_private_data gromit_data;

/*-------------------------------------------------------------------------*/
// Interface decriptor (used for FS and HS descrptor tables)
static struct usb_interface_descriptor gromit_intf = 
{
   .bLength =  sizeof(gromit_intf),
   .bDescriptorType = USB_DT_INTERFACE,
   .bNumEndpoints = 3,
   .bInterfaceClass = USB_CLASS_VENDOR_SPEC,
   /* .iInterface = DYNAMIC */
};

/*-------------------------------------------------------------------------*/
/* full speed support: */
static struct usb_endpoint_descriptor fs_data_in_desc = 
{
   .bLength =  USB_DT_ENDPOINT_SIZE,
   .bDescriptorType = USB_DT_ENDPOINT,
   .bEndpointAddress = USB_DIR_IN,
   .bmAttributes =  USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor fs_ctrl_out_desc = 
{
   .bLength =  USB_DT_ENDPOINT_SIZE,
   .bDescriptorType = USB_DT_ENDPOINT,
   .bEndpointAddress = USB_DIR_OUT,
   .bmAttributes =  USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor fs_ctrl_in_desc = 
{
   .bLength =  USB_DT_ENDPOINT_SIZE,
   .bDescriptorType = USB_DT_ENDPOINT,
   .bEndpointAddress = USB_DIR_IN,
   .bmAttributes =  USB_ENDPOINT_XFER_BULK,
};

static struct usb_descriptor_header *fs_descs[] = 
{
   (struct usb_descriptor_header *) &gromit_intf,
   (struct usb_descriptor_header *) &fs_ctrl_out_desc,
   (struct usb_descriptor_header *) &fs_ctrl_in_desc,
   (struct usb_descriptor_header *) &fs_data_in_desc,
   NULL,
};

/*-------------------------------------------------------------------------*/
/* high speed support: */

// Size of transfers
#define BULK_SIZE (512)

static struct usb_endpoint_descriptor hs_data_in_desc = 
{
   .bLength =  USB_DT_ENDPOINT_SIZE,
   .bDescriptorType = USB_DT_ENDPOINT,
   //.bEndpointAddress = allocated in function
   .bmAttributes =  USB_ENDPOINT_XFER_BULK,
   .wMaxPacketSize = cpu_to_le16(BULK_SIZE),
};

static struct usb_endpoint_descriptor hs_ctrl_out_desc = 
{
   .bLength =  USB_DT_ENDPOINT_SIZE,
   .bDescriptorType = USB_DT_ENDPOINT,
   //.bEndpointAddress = allocated in function
   .bmAttributes =  USB_ENDPOINT_XFER_BULK,
   .wMaxPacketSize = cpu_to_le16(BULK_SIZE),
};

static struct usb_endpoint_descriptor hs_ctrl_in_desc = 
{
   .bLength =  USB_DT_ENDPOINT_SIZE,
   .bDescriptorType = USB_DT_ENDPOINT,
   //.bEndpointAddress = allocated in function
   .bmAttributes =  USB_ENDPOINT_XFER_BULK,
   .wMaxPacketSize = cpu_to_le16(BULK_SIZE),
};

static struct usb_descriptor_header *hs_descs[] = 
{
   (struct usb_descriptor_header *) &gromit_intf,
   (struct usb_descriptor_header *) &hs_ctrl_out_desc,
   (struct usb_descriptor_header *) &hs_ctrl_in_desc,
   (struct usb_descriptor_header *) &hs_data_in_desc,
   NULL,
};

/*-------------------------------------------------------------------------*/
// Strings for the USB function

static struct usb_string function_str[] = 
{
   [0].s = "default",
   {  }   /* end of list */
};

static struct usb_gadget_strings function_str_table = 
{
   .language = 0x0409, /* en-us ALERT AY */
   .strings = function_str,
};

static struct usb_gadget_strings *function_str_tablelist[] = 
{
   &function_str_table,
   NULL,
};

static void bulk_ep_complete(struct usb_ep *ep, struct usb_request *req);
static struct usb_request *gromit_alloc_ep_req(struct usb_ep *ep, int len);

/*-------------------------------------------------------------------------*/
static struct usb_request *gromit_alloc_ep_req(struct usb_ep *ep, int len)
{
   struct usb_request *req = NULL;

   PDEBUG("alloc ep req for %p, len 0x%04X\n", ep, len);

   req = usb_ep_alloc_request(ep, GFP_ATOMIC);
   if (req) 
   {
      req->length = len;
      req->buf = kmalloc(len, GFP_ATOMIC);
      if (!req->buf) 
      {
         usb_ep_free_request(ep, req);
         req = NULL;
      }
      PDEBUG ("gromit allocated %p for ep req\n", req);
   }

   return req;
}

/*-------------------------------------------------------------------------*/
// Callback for BULK endpoint completion interrupt
static void bulk_ep_complete(struct usb_ep *ep, struct usb_request *req)
{
   struct gromit_private_data *data;
   struct gromit_devfile *file = NULL;
   struct ep_req *t_req;
   unsigned long flags;
   bool out = false;
   struct list_head *pending_head = NULL;

   data = (struct gromit_private_data *) ep->driver_data;
   t_req = (struct ep_req *) req->context;

   PDEBUG("gromit ep=%p, req=%p, data=%p, ep_req=%p\n", 
          ep, req, data, t_req);

   if (!data || !t_req)
      return;

   // Locate device file info
   if (data->data_dev.in_ep.ep == ep)
   {
      file = &data->data_dev;
      pending_head = &file->in_ep.pending_req;
      out = false;
   }
   else if (data->ctrl_dev.out_ep.ep == ep)
   {
      file = &data->ctrl_dev;
      pending_head = &file->out_ep.pending_req;
      out = true;
   }
   else if (data->ctrl_dev.in_ep.ep == ep)
   {
      file = &data->ctrl_dev;
      pending_head = &file->in_ep.pending_req;
      out = false;
   }
   else
   {
      // Error!
      PDEBUG("*** Bad descriptor!\n");
      return;
   }

   PDEBUG("%s: Pending reqs: %s\n", out ? "OUT" : "IN",
          list_empty(pending_head) ? "empty" : "not empty");

   if (!out)
   {
      PDEBUG("IN packet ep=%p, req=%p, len=%04X\n", ep, req, req->actual);

      // IN request. We just delete the request and then move the 
      // container back to the free queue
      kfree(req->buf);
      usb_ep_free_request(ep, req);
      t_req->req = NULL;

      spin_lock_irqsave(&file->lock, flags);
      list_del_init(&t_req->node);
      list_add(&t_req->node, &file->in_ep.free_req);

      // Keep track of number of elements available in free_req list
      file->in_ep.free_reqs++;
      if (file->in_ep.free_reqs > file->in_ep.max_free_reqs)
      {
         file->in_ep.max_free_reqs = file->in_ep.free_reqs;
      }

      spin_unlock_irqrestore(&file->lock, flags);
   }
   else if (req->status < 0)
   {
      PDEBUG("OUT packet, ep=%p, status=%d\n", ep, req->status);

      // OUT request connection shut down or aborted. Just transfer
      // the request to the free queue
      spin_lock_irqsave(&file->lock, flags);

      list_del_init(&t_req->node);
      list_add(&t_req->node, &file->out_ep.free_req);

      spin_unlock_irqrestore(&file->lock, flags);
   }
   else
   {
      PDEBUG("OUT packet, ep=%p, status=%d, len=%04X\n", 
             ep, req->status, req->actual);

      // Completion callback. Put the data onto the 'collect' queue
      spin_lock_irqsave(&file->lock, flags);

      // reset 'offset' field of t_req before the fop_read() function
      // starts
      t_req->offset = 0;

      list_del_init(&t_req->node);
      list_add(&t_req->node, &file->out_ep.collect_req);
      file->out_ep.collect_data = true;

      spin_unlock_irqrestore(&file->lock, flags);
      PDEBUG("ep %p, collect count=%d\n", &file->out_ep, 
             file->out_ep.collect_data);
   }

   // Wake up anything that's waiting for data
   wake_up_interruptible(&file->wq);
}

/*-------------------------------------------------------------------------*/
// File operations, shared between data/ctrl endpoints
static int fop_open(struct inode* ino, struct file* f)
{
   struct gromit_devfile *data;

   PDEBUG ("gromit device open, filp %p\n", f);

   // Get driver structure corresponding to this inode
   data = container_of(ino->i_cdev, struct gromit_devfile, cdev);
   f->private_data = (void*) data;

   if (!atomic_dec_and_test(&data->in_use))
   {
      atomic_inc(&data->in_use);
      return -EBUSY;
   }

   if (down_interruptible(&data->sem))
      return -ERESTARTSYS;

   // Put requests on the queue for OUT endpoints
   if (data->out_ep.ep != NULL)
   {
      struct ep_req *ptr, *next;
      unsigned long flags;

      spin_lock_irqsave(&data->lock, flags);
      
      // Put all reqs in 'collect' back into 'free', in case
      // there's data left over from the last time the file was 
      // opened
      list_for_each_entry_safe(ptr, next, 
                               &data->out_ep.collect_req, node)
      {
         list_del_init(&ptr->node);
         list_add(&ptr->node, &data->out_ep.free_req);
      }

      // Submit all the free requests
      list_for_each_entry_safe(ptr, next, 
                               &data->out_ep.free_req, node)
      {
         int status;

         // Allocate request now if need be
         if (ptr->req == NULL)
         {
            ptr->req = gromit_alloc_ep_req(data->out_ep.ep, REQ_LEN);
         }

         if (ptr->req == NULL)
         {
            break;
         }

         // All OUT endpoints in this driver are BULK
         ptr->req->complete = bulk_ep_complete;
         ptr->req->context = ptr;
         status = usb_ep_queue(data->out_ep.ep, ptr->req, GFP_ATOMIC);
         if (status == 0)
         {
            // Put the request on the 'pending' list
            list_del_init(&ptr->node);
            list_add(&ptr->node, &data->out_ep.pending_req);
         }
         else
         {
            // If the peripheral is not connected, for example, then
            // an error will be generated
            dev_err(data->dev, "open OUT %s --> %d\n",
                    data->out_ep.ep->name, status);

            break;
         }
      }

      // Reset count of packets waiting collection
      data->out_ep.collect_data = false;

      spin_unlock_irqrestore(&data->lock, flags);
   }

   // Free semaphore
   up(&data->sem);

   PDEBUG ("gromit device opened\n");

   return 0;
}

/*-------------------------------------------------------------------------*/
static int fop_close(struct inode* ino, struct file* f)
{
   struct gromit_devfile *data = (struct gromit_devfile *) f->private_data;

   PDEBUG ("gromit device close filp=%p, data=%p\n", f, data);

   if (down_interruptible(&data->sem))
      return -ERESTARTSYS;

   // Don't release requests here. If the file is re-opened the driver
   // can pick up where it left off, and if the driver is suspended
   // or removed the requests will be released elsewhere in any case.

   // Free semaphore
   up(&data->sem);

   // Flag the device as free for re-opening
   atomic_inc(&data->in_use);

   return 0;
}

/*-------------------------------------------------------------------------*/
static ssize_t send_data(struct gromit_devfile *data,
                         const char __user *buf, size_t length)
{
   unsigned long flags;
   ssize_t ret = 0;
   struct ep_req *ptr;

   // Get first entry in free request list
   // Use spinlock in case a packet arrives while we're busy
   spin_lock_irqsave(&data->lock, flags);
      
   if ((buf == NULL) || (length == 0) || list_empty(&data->in_ep.free_req))
   {
      PDEBUG("Nothing to send: len=0x%04X\n", length);

      spin_unlock_irqrestore(&data->lock, flags);
      return 0;
   }

   PDEBUG("b=%p l=%04X tl=%04X\n", buf, length, length);

   ptr = list_first_entry(&data->in_ep.free_req, struct ep_req, node);
   list_del_init(&ptr->node);

   // Keep track of number of elements available in free_req list
   data->in_ep.free_reqs--;
   if (data->in_ep.free_reqs < data->in_ep.min_free_reqs)
   {
      data->in_ep.min_free_reqs = data->in_ep.free_reqs;
   }

   spin_unlock_irqrestore(&data->lock, flags);

   // Allocate data for the transfer
   ptr->req = gromit_alloc_ep_req(data->in_ep.ep, length);
   if (ptr->req != NULL)
   {
      PDEBUG("Copying user memory into USB req\n");

      // Set up fields
      ptr->req->complete = bulk_ep_complete;
      ptr->req->context = ptr;
      ptr->req->zero = 1;

      // 'buf' may be NULL, if we're just flushing out pending data
      if (!copy_from_user(ptr->req->buf, buf, length))
      {
         // Put the request on the 'pending' list
         spin_lock_irqsave(&data->lock, flags);
         list_add(&ptr->node, &data->in_ep.pending_req);
         spin_unlock_irqrestore(&data->lock, flags);

         // Try to queue request
         ret = usb_ep_queue(data->in_ep.ep, ptr->req, GFP_ATOMIC);
         if (ret == 0)
         {
            PDEBUG("Queued USB req\n");
            ret = length;
         }
         else
         {
            // If the peripheral is not connected, for example, then
            // an error will be generated
            dev_err(data->dev, "send IN %s --> %d\n",
                    data->in_ep.ep->name, ret);

            // Free the memory (we're not using it...)
            kfree(ptr->req->buf);
            usb_ep_free_request(data->in_ep.ep, ptr->req);
            ptr->req = NULL;

            // Put request back on 'free' queue
            spin_lock_irqsave(&data->lock, flags);
            list_del_init(&ptr->node);
            list_add(&ptr->node, &data->in_ep.free_req);
            spin_unlock_irqrestore(&data->lock, flags);
         }
      }
      else
      {
         ret = -EFAULT;
      }
   }
   else
   {
      ret = -ENOMEM;
   }

   return ret;
}

/*-------------------------------------------------------------------------*/
static ssize_t fop_write(struct file* f, const char __user *buf, 
                         size_t length, loff_t* offset)
{
   ssize_t ret = 0;
   struct gromit_devfile *data = (struct gromit_devfile *) f->private_data;

   PDEBUG("fop_write(): file %p, ep = %p, len=%04X\n", 
          data, data->in_ep.ep, length);

   if (down_interruptible(&data->sem))
      return -ERESTARTSYS;
  
   // Wait for a request to become available
   while (list_empty(&data->in_ep.free_req))
   {
      // Release semaphore
      up(&data->sem);

      // If non-blocking, quit now
      if (f->f_flags & O_NONBLOCK)
         return -EAGAIN;

      // Wait for a request to be freed
      if (wait_event_interruptible(data->wq, 
                                   (!list_empty(&data->in_ep.free_req))))
      {
         PDEBUG("Woken by signal - returning\n");
         return -ERESTARTSYS;
      }
   
      // Re-acquire lock
      if (down_interruptible(&data->sem))
         return -ERESTARTSYS;
   }

   if (length == 0)
   {
      PDEBUG("Total TX requested is zero\n");
   }
   else 
   {
      ret = send_data(data, buf, length);

      PDEBUG("Sent total=%04X bytes\n", ret);
   }

   // Release semaphore
   up(&data->sem);

   PDEBUG("fop_write: ret = 0x%04X\n", ret);

   return ret;
}

/*-------------------------------------------------------------------------*/
static ssize_t fop_read(struct file* f, char __user *buf, 
                        size_t length, loff_t* offset)
{
   ssize_t ret = 0;
   struct gromit_devfile *data = (struct gromit_devfile *) f->private_data;
   unsigned long flags;
   struct ep_req *ptr, *next;
   int len;

   if (down_interruptible(&data->sem))
      return -ERESTARTSYS;

   //PDEBUG("fop_read(): User buf=%p, length=%04X, off=%04X\n",
   //       buf, length, (unsigned int) *offset);

   while (list_empty(&data->out_ep.user_req) 
          && !data->out_ep.collect_data)
   {
      // Release semaphore
      up(&data->sem);

      // If non-blocking, quit now
      if (f->f_flags & O_NONBLOCK)
         return -EAGAIN;

      // Wait for data to arrive
      if (wait_event_interruptible(data->wq, data->out_ep.collect_data))
      {
         PDEBUG("Woken by signal - returning\n");
         return -ERESTARTSYS;
      }
   
      // Re-acquire lock
      if (down_interruptible(&data->sem))
         return -ERESTARTSYS;
   }

   // We now have data ready to read. Copy reqs into separate list so that
   // we can process them without needing a spinlock
   spin_lock_irqsave(&data->lock, flags);
   list_for_each_entry_safe(ptr, next, &data->out_ep.collect_req, node)
   {
      // Remove from list of buffers pending collection
      list_del_init(&ptr->node);
      
      // Put the request on the 'user' list
      list_add(&ptr->node, &data->out_ep.user_req);
   }
   data->out_ep.collect_data = false;
   spin_unlock_irqrestore(&data->lock, flags);

   // Count of data copied to user so far
   len = 0;

   list_for_each_entry_safe(ptr, next, &data->out_ep.user_req, node)
   {
      int status;
      int this_len;
      uint8_t *tp;

      PDEBUG("RX: len=%04X, req=%04X, left=%04X, length=%04X\n",
            len, ptr->req->actual, 
            ptr->req->actual - ptr->offset, length);

      // The length to be copied from this req is reduced by the 
      // offset - i.e. the amount we've sent in a previous read
      this_len = ptr->req->actual - ptr->offset;
      if (len + this_len > length)
         this_len = length - len;

      // Quit if we can't copy anything at all
      if (this_len == 0)
         break;

      tp = (uint8_t*) ptr->req->buf;
      PDEBUG("%02X %02X %02X %02X %02X %02X %02X %02X\n", 
             tp[ptr->offset], tp[ptr->offset + 1],
             tp[ptr->offset + 2], tp[ptr->offset + 3],
             tp[ptr->offset + 4], tp[ptr->offset + 5],
             tp[ptr->offset + 6], tp[ptr->offset + 7]);

      PDEBUG("Copying %04X bytes\n", this_len);
      if (copy_to_user(buf, ptr->req->buf + ptr->offset, this_len) != 0)
      {
         PDEBUG("copy_to_user failed\n");
         ret = -EFAULT;
         break;
      }

      len += this_len;
      buf += this_len;
      ret += this_len;

      if (this_len == ptr->req->actual - ptr->offset)
      {
         PDEBUG("Finished with buffer\n");

         // Remove from list of buffers pending collection
         list_del_init(&ptr->node);
         
         // Take spinlock while accessing lists
         spin_lock_irqsave(&data->lock, flags);

         // Try to re-queue request
         status = usb_ep_queue(data->out_ep.ep, ptr->req, GFP_ATOMIC);
         if (status == 0)
         {
            // Put the request on the 'pending' list
            list_add(&ptr->node, &data->out_ep.pending_req);
         }
         else
         {
            // If the peripheral is not connected, for example, then
            // an error will be generated
            dev_err(data->dev, "start OUT %s --> %d\n",
                    data->out_ep.ep->name, status);
            
            // Put the request on the 'free' list
            list_add(&ptr->node, &data->out_ep.free_req);         
         }

         spin_unlock_irqrestore(&data->lock, flags);
      }
      else
      {
         PDEBUG("Cacheing remainder of buffer (%04X)\n", this_len);

         // only copied a fraction of the data in the buffer. Keep it for
         // next time. And no need to go round the loop again, since we've
         // already filled up the user buffer
         ptr->offset += this_len;
         break;
      }
   }

   // Free semaphore
   up(&data->sem);

   return ret;
}

//----------------------------------------------------------------------------
static unsigned int fop_poll(struct file *f, poll_table *wait)
{
   struct gromit_devfile *data;
   unsigned int mask = 0;
   unsigned long flags;

   data = (struct gromit_devfile *) f->private_data;
   down(&data->sem);
   poll_wait(f, &data->wq, wait);

   spin_lock_irqsave(&data->lock, flags);

   if ((data->out_ep.ep != NULL) && (data->out_ep.collect_data))
   {
      mask |= POLLIN | POLLRDNORM;
   }
   if (data->in_ep.ep != NULL)
   {
      mask |= POLLOUT | POLLWRNORM;
   }

   spin_unlock_irqrestore(&data->lock, flags);

   up(&data->sem);

   return mask;
}

/*-------------------------------------------------------------------------*/
static struct file_operations fops = 
{
 .open = fop_open,
 .release = fop_close,
 .read = fop_read,
 .write = fop_write,
 .poll = fop_poll,
};

/*-------------------------------------------------------------------------*/
static void disable_ep(struct usb_composite_dev *cdev, struct usb_ep *ep)
{
   int   value;

   if (ep->driver_data) 
   {
      value = usb_ep_disable(ep);
      if (value < 0)
         DBG(cdev, "disable %s --> %d\n", ep->name, value);

      ep->driver_data = NULL;
   }
}

/*-------------------------------------------------------------------------*/
static void disable_data_endpoints(struct gromit_private_data *data)
{
   struct usb_composite_dev *cdev;
   
   cdev = data->function.config->cdev;

   if (data->data_dev.in_ep.ep != NULL)
   {
      disable_ep(cdev, data->data_dev.in_ep.ep);
   }

   if (data->ctrl_dev.out_ep.ep != NULL)
   {
      disable_ep(cdev, data->ctrl_dev.out_ep.ep);
   }

   if (data->ctrl_dev.in_ep.ep != NULL)
   {
      disable_ep(cdev, data->ctrl_dev.in_ep.ep);
   }

   VDBG(cdev, "%s disabled\n", data->function.name);
}

/*-------------------------------------------------------------------------*/
static int
enable_endpoint(struct gromit_private_data *data, struct usb_ep *ep)
{
   int result = 0;
   struct usb_composite_dev *cdev;
   
   cdev = data->function.config->cdev;

   ep->driver_data = NULL;
   result = config_ep_by_speed(cdev->gadget, &(data->function), ep);
   PDEBUG ("config ep %p by speed = %d\n", ep, result);
   if (result == 0)
   {
      result = usb_ep_enable(ep);
      PDEBUG ("enable ep  = %d\n", result);
      if (result == 0)
      {
         ep->driver_data = data;
      }
   }

   return result;
}

/*-------------------------------------------------------------------------*/
static int
enable_data_endpoints(struct gromit_private_data *data)
{
   int result = 0;

   // DATA IN endpoint
   result = enable_endpoint(data, data->data_dev.in_ep.ep);
   if (result < 0)
   {
      goto err_out;
   }

   // CTRL OUT endpoint
   result = enable_endpoint(data, data->ctrl_dev.out_ep.ep);
   if (result < 0)
   {
      goto err1;
   }

   // CTRL IN endpoint
   result = enable_endpoint(data, data->ctrl_dev.in_ep.ep);
   if (result < 0)
   {
      goto err2;
   }
   
   PDEBUG ("All good\n");
   
   return result;

 err2:
   PDEBUG ("Tidy CTRL OUT\n");
   usb_ep_disable(data->ctrl_dev.out_ep.ep);
   data->ctrl_dev.out_ep.ep->driver_data = NULL;

 err1:
   PDEBUG ("Tidy CTRL IN \n");
   usb_ep_disable(data->ctrl_dev.in_ep.ep);
   data->ctrl_dev.in_ep.ep->driver_data = NULL;
      
 err_out:
   return result;
}

/*-------------------------------------------------------------------------*/
static void __init init_ep_struct(struct gromit_ep_data *ep)
{
   ep->ep = NULL;

   INIT_LIST_HEAD(&ep->pending_req);
   INIT_LIST_HEAD(&ep->free_req);
   INIT_LIST_HEAD(&ep->collect_req);
   INIT_LIST_HEAD(&ep->user_req);

   ep->collect_data = false;
   ep->free_reqs = 0;
   ep->min_free_reqs = 0;
   ep->max_free_reqs = 0;
}

/*-------------------------------------------------------------------------*/
static int __init init_ep_data(struct usb_composite_dev *cdev,
                               struct gromit_ep_data *ep,
                               struct usb_endpoint_descriptor *desc,
                               uint32_t num_reqs)
{
   int ret = -1;

   // Initialise queues etc.
   init_ep_struct(ep);
   ep->free_reqs = num_reqs;
   ep->min_free_reqs = num_reqs;
   ep->max_free_reqs = num_reqs;

   /* allocate endpoints */
   ep->ep = usb_ep_autoconfig(cdev->gadget, desc);
   if (ep->ep)
   {
      uint32_t i;

      ep->ep->driver_data = cdev;

      PDEBUG("init_ep_data: allocating %d reqs\n", num_reqs);
      for(i = 0; i < num_reqs; i++)
      {
         struct ep_req *req;

         req = kzalloc(sizeof(struct ep_req), GFP_KERNEL);
         if (req == NULL)
            break;

         // Add new (empty) request structure to list of free items
         list_add_tail(&req->node, &ep->free_req);
      }

      if (i == num_reqs)
         ret = 0;
   }

   PDEBUG("init_ep_data: h=%p, n=%p, p=%p\n", 
          &ep->free_req,
          ep->free_req.next,
          ep->free_req.prev);

   return ret;
}

/*-------------------------------------------------------------------------*/
static int __init
function_bind(struct usb_configuration *c, struct usb_function *f)
{
   struct usb_composite_dev *cdev = c->cdev;
   struct gromit_private_data *data;
   int id;

   // Get pointer to private data
   data = container_of(f, struct gromit_private_data, function);

   /* allocate interface ID(s) */
   id = usb_interface_id(c, f);
   if (id < 0)
      return id;

   // Set ID in interface descriptor header
   gromit_intf.bInterfaceNumber = id;

   f->fs_descriptors = fs_descs;

   /* allocate endpoints. Data file has one IN endpoint,
   * Ctrl file has one OUT and one IN endpoint*/
   if (init_ep_data(cdev, &data->ctrl_dev.out_ep, 
                    &fs_ctrl_out_desc, NUM_CTRL_REQS) < 0)
   {
      goto autoconf_fail;
   }

   if (init_ep_data(cdev, &data->ctrl_dev.in_ep, 
                    &fs_ctrl_in_desc, NUM_CTRL_REQS) < 0)
   {
      goto autoconf_fail;
   }

   // Initialise queues etc. for data OUT, even though we won't use it
   // (makes sure that reads of the OUT data don't crash the system)
   init_ep_struct(&data->data_dev.out_ep);

   if (init_ep_data(cdev, &data->data_dev.in_ep, 
                    &fs_data_in_desc, NUM_DATA_REQS) < 0)
   {
      goto autoconf_fail;
   }

   PDEBUG("eps: CTRL_IN=%p, CTRL_OUT=%p, DATA_IN=%p\n",
          data->ctrl_dev.in_ep.ep,
          data->ctrl_dev.out_ep.ep,
          data->data_dev.in_ep.ep);
          
   /* support high speed hardware */
   if (gadget_is_dualspeed(c->cdev->gadget)) 
   {
      hs_data_in_desc.bEndpointAddress =
         fs_data_in_desc.bEndpointAddress;
      hs_ctrl_out_desc.bEndpointAddress =
         fs_ctrl_out_desc.bEndpointAddress;
      hs_ctrl_in_desc.bEndpointAddress =
         fs_ctrl_in_desc.bEndpointAddress;
      f->hs_descriptors = hs_descs;
   }

   if (gadget_is_superspeed(c->cdev->gadget))
   {
      PDEBUG ("Gadget is superspeed!\n");
   }

   PDEBUG( "%s speed %s: Data IN/%s, Ctrl OUT/%s Ctrl IN/%s\n",
           gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full", f->name, 
           data->data_dev.in_ep.ep->name, 
           data->ctrl_dev.out_ep.ep->name, 
           data->ctrl_dev.in_ep.ep->name);

   return 0;

 autoconf_fail:
   ERROR(cdev, "%s: can't autoconfigure on %s\n",
         f->name, cdev->gadget->name);
   return -ENODEV;
}

/*-------------------------------------------------------------------------*/
static void free_ep_data(struct usb_ep *ep, struct list_head *req_list)
{
   struct ep_req *ptr, *next;
   /* int i = 0; */

   if (list_empty(req_list))
   {
      PDEBUG("List empty\n");
      return;
   }

   PDEBUG("free_ep_data: h=%p, n=%p, p=%p\n", 
          req_list, req_list->next, req_list->prev);

   list_for_each_entry_safe(ptr, next, req_list, node)
   {
      //PDEBUG("Freeing req %d %p: %p\n", i++, ptr, ptr->req);
      if (ptr->req != NULL)
      {
         if (ptr->req->buf != NULL)
            kfree(ptr->req->buf);

         usb_ep_free_request(ep, ptr->req);
      }

      // Remove from list
      list_del(&ptr->node);

      // Release data
      kfree(ptr);
   }

   PDEBUG("Freed Reqs\n");
}

/*-------------------------------------------------------------------------*/
static void
function_unbind(struct usb_configuration *c, struct usb_function *f)
{
   struct gromit_private_data *data;

   PDEBUG("gromit unbind\n");

   // Get pointer to private data
   data = container_of(f, struct gromit_private_data, function);

   PDEBUG("Freeing CTRL OUT reqs\n");
   free_ep_data(data->ctrl_dev.out_ep.ep, &data->ctrl_dev.out_ep.free_req);
   free_ep_data(data->ctrl_dev.out_ep.ep, &data->ctrl_dev.out_ep.pending_req);
   PDEBUG("Freed CTRL OUT reqs\n");

   PDEBUG("Freeing CTRL IN reqs\n");
   free_ep_data(data->ctrl_dev.in_ep.ep, &data->ctrl_dev.in_ep.free_req);
   free_ep_data(data->ctrl_dev.in_ep.ep, &data->ctrl_dev.in_ep.pending_req);
   PDEBUG("Freed CTRL IN reqs\n");

   PDEBUG("Freeing DATA IN reqs\n");
   free_ep_data(data->data_dev.in_ep.ep, &data->data_dev.in_ep.free_req);
   free_ep_data(data->data_dev.in_ep.ep, &data->data_dev.in_ep.pending_req);
   PDEBUG("Freed DATA IN reqs\n");
}

/*-------------------------------------------------------------------------*/
static int function_set_alt(struct usb_function *f,
                            unsigned intf, unsigned alt)
{
   struct gromit_private_data *data;
   int ret;

   PDEBUG("usb_function=%p intf=%u alt=%u\n",
          f, intf, alt);

   if (f == NULL)
   {
      PDEBUG("usb_function is NULL");
      return -EINVAL;
   }

   // Get pointer to private data
   data = container_of(f, struct gromit_private_data, function);
   if (data == NULL)
   {
      PDEBUG("private_data is NULL");
      return -EINVAL;
   }

   /* we know alt is zero */
   if (data->ctrl_dev.in_ep.ep->driver_data)
   {
      disable_data_endpoints(data);
   }

   ret = enable_data_endpoints(data);
   PDEBUG ("enable source sink = %d\n", ret);

   return ret;
}

/*-------------------------------------------------------------------------*/
static void function_disable(struct usb_function *f)
{
   struct gromit_private_data *data;

   // Get pointer to private data
   data = container_of(f, struct gromit_private_data, function);

   PDEBUG ("gromit disable\n");

   disable_data_endpoints(data);
}

/*-------------------------------------------------------------------------*/
static void gromit_unbind_config(struct usb_configuration *c)
{
   PDEBUG ("gromit unbind configuration\n");
}

/*-------------------------------------------------------------------------*/
static int __init gromit_bind_config(struct usb_configuration *c)
{
   gromit_data.function.name = "gromit";
   gromit_data.function.bind = function_bind;
   gromit_data.function.unbind = function_unbind;
   gromit_data.function.set_alt = function_set_alt;
   gromit_data.function.disable = function_disable;
   gromit_data.function.os_desc_table = &gromit_os_desc_table;
   gromit_data.function.os_desc_n = 1;

   INIT_LIST_HEAD(&gromit_os_desc.ext_prop);
   list_add_tail(&gromit_os_desc_prop_guid.entry, &gromit_os_desc.ext_prop);

   c->cdev->b_vendor_code = vendor_code;
   c->cdev->os_desc_config = c;
   utf8s_to_utf16s(os_string_desc, sizeof(os_string_desc), UTF16_LITTLE_ENDIAN,
   			(wchar_t *)c->cdev->qw_sign, OS_STRING_QW_SIGN_LEN);
   c->cdev->use_os_string = 1;

   return usb_add_function(c, &gromit_data.function);
}

/*-------------------------------------------------------------------------*/
static struct usb_configuration gromit_config = 
{
   .label = "gromit",
   .strings = function_str_tablelist,
   .unbind = gromit_unbind_config,
   .bConfigurationValue = 3,
   .bmAttributes = USB_CONFIG_ATT_SELFPOWER,
   /* .iConfiguration = DYNAMIC */
};

/*-------------------------------------------------------------------------*/
static int __init gromit_build_device(struct gromit_private_data *data,
                                      struct gromit_devfile *info,
                                      int dev_offset,
                                      const char *name)
{
   info->in_ep.ep = NULL;
   info->out_ep.ep = NULL;

   // Create a device file and cdev object
   info->dev = device_create(data->class, NULL, data->devnum + dev_offset, 
                             (void*) data, "%s_%s", DRIVER_NAME, name);
   if (info->dev == NULL) 
      goto err_out;

   PDEBUG("Name=%s, data=%p info=%p\n", name, data, info);

   PDEBUG("name=%s, major %d\n", name, MAJOR(data->devnum + 1));
   cdev_init(&info->cdev, &fops);

   if (cdev_add(&info->cdev, data->devnum + dev_offset, 1) == -1) 
      goto err1;

   info->cdev.owner = THIS_MODULE;

   // Initialise locks etc. for handling access
   init_waitqueue_head(&info->wq);
   spin_lock_init(&info->lock);
   sema_init(&info->sem, 1);
   atomic_set(&info->in_use, 1);

   PDEBUG("name=%s, file=%p, id=%08X, dev_id=%08X\n", name, info,
          data->devnum + dev_offset, info->dev->devt);

   return 0;

 err1:
   device_destroy(data->class, data->devnum + dev_offset);

 err_out:

   return -1;
}

/*-------------------------------------------------------------------------*/
int __init gromit_add_devices(struct usb_composite_dev *usb_cdev)
{
   int ret = -EINVAL;
   int id;

   // Create device class and devices first, so that we can 
   // store EP data in device file structures
   gromit_data.class = class_create(THIS_MODULE, DRIVER_NAME);
   if (gromit_data.class == NULL) 
      goto err_out;
   
   PDEBUG("Created class %p\n", gromit_data.class);

   /* device constructor. Need two devices - one for control and one
    * for data */
   if (alloc_chrdev_region(&(gromit_data.devnum), 0, 2, DRIVER_NAME) < 0)
      goto err1;
   
   PDEBUG("Allocated chrdev major=%d\n", MAJOR(gromit_data.devnum));

   // Build data device
   if (gromit_build_device(&gromit_data, &gromit_data.data_dev, 
                           0, "data") < 0)
      goto err1;

   PDEBUG("Allocated data device\n");

   // Build ctrl device
   if (gromit_build_device(&gromit_data, &gromit_data.ctrl_dev, 
                           1, "ctrl") < 0)
      goto err2;
   
   PDEBUG("Allocated ctrl device\n");

   // Allocate string ID(s)
   id = usb_string_id(usb_cdev);
   if (id < 0)
      goto err2;

   function_str[0].id = id;

   PDEBUG("Allocated string ID %d\n", id);

   gromit_config.iConfiguration = id;
   ret = usb_add_config(usb_cdev, &gromit_config, gromit_bind_config);
   if (ret < 0)
      goto err2;

   PDEBUG("Added gromit USB configuration\n");

   return ret;

 err2:
   cdev_del(&gromit_data.data_dev.cdev);
   device_destroy(gromit_data.class, gromit_data.devnum);

 err1:
   class_destroy(gromit_data.class);
   gromit_data.class = NULL;

 err_out:
   return ret;
}

/*-------------------------------------------------------------------------*/
static void gromit_comp_driver_suspend(struct usb_composite_dev *cdev)
{
   if (cdev->gadget->speed == USB_SPEED_UNKNOWN)
      return;

   DBG(cdev, "%s\n", __func__);
}

/*-------------------------------------------------------------------------*/
static void gromit_comp_driver_resume(struct usb_composite_dev *cdev)
{
   DBG(cdev, "%s\n", __func__);
}

/*-------------------------------------------------------------------------*/
static ssize_t attr_show_stats(struct device *dev,
                               struct device_attribute *attr, char *buf)
{
   int rc;
   struct gromit_ep_data *ep;

   rc = scnprintf(buf, PAGE_SIZE, "Free requests:\n");

   ep = &gromit_data.data_dev.in_ep;
   rc += scnprintf(buf + rc, PAGE_SIZE - rc, 
                   "DATA IN: Min=%d, Max=%d, Current=%d\n",
                   ep->min_free_reqs,
                   ep->max_free_reqs,
                   ep->free_reqs);
                      
   ep = &gromit_data.ctrl_dev.in_ep;
   rc += scnprintf(buf + rc, PAGE_SIZE - rc, 
                   "CTRL IN: Min=%d, Max=%d, Current=%d\n",
                   ep->min_free_reqs,
                   ep->max_free_reqs,
                   ep->free_reqs);
                      
   ep = &gromit_data.ctrl_dev.out_ep;
   rc += scnprintf(buf + rc, PAGE_SIZE - rc, 
                   "CTRL OUT: Min=%d, Max=%d, Current=%d\n",
                   ep->min_free_reqs,
                   ep->max_free_reqs,
                   ep->free_reqs);
                      
   return rc;
}

/*-------------------------------------------------------------------------*/
static struct usb_device_descriptor device_desc = 
{
   .bLength = sizeof(device_desc),
   .bDescriptorType = USB_DT_DEVICE,
   
   .bcdUSB = cpu_to_le16(0x0200),
   .bDeviceClass = USB_CLASS_VENDOR_SPEC,
   
   .idVendor = cpu_to_le16(DRIVER_VENDOR_NUM_DEF),
   .idProduct = cpu_to_le16(DRIVER_PRODUCT_NUM_DEF),
   .bNumConfigurations = 1,
};

/*-------------------------------------------------------------------------*/
// Strings for the device (As opposed to strings for individual functions)
#define STRING_MANUFACTURER_IDX  0
#define STRING_PRODUCT_IDX       1
#define STRING_SERIAL_IDX        2

static char *manufacturer __initdata = "RF Creations";
static char *longname __initdata = "Moreph20";

/* default serial number takes at least two packets */
static char *iSerialNumber __initdata = "0123456789.0123456789.0123456789";

module_param( iSerialNumber , charp , S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP );
module_param( manufacturer , charp , S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP );
module_param( longname , charp , S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP );

static struct usb_string device_str[] = 
{
   [STRING_MANUFACTURER_IDX].s = NULL, //manufacturer,
   [STRING_PRODUCT_IDX].s = NULL, //longname,
   [STRING_SERIAL_IDX].s = NULL,
   {  }   /* end of list */
};

static struct usb_gadget_strings device_str_table = 
{
   .language = 0x0409, /* en-us */
   .strings = device_str,
};

static struct usb_gadget_strings *device_str_tablelist[] = 
{
   &device_str_table,
   NULL,
};

DEVICE_ATTR(req_stats, S_IRUGO, attr_show_stats, NULL);

/*-------------------------------------------------------------------------*/
static int __init gromit_comp_driver_bind(struct usb_composite_dev *cdev)
{
   int   gcnum;
   struct usb_gadget *gadget = cdev->gadget;
   int   id;

   /* Allocate string descriptor numbers ... note that string
    * contents can be overridden by the composite_dev glue.
    */

   device_desc.idVendor = cpu_to_le16(DRIVER_VENDOR_NUM);
   device_desc.idProduct = cpu_to_le16(DRIVER_PRODUCT_NUM);


   PDEBUG ("GROMIT BIND\n");
   // Initialise the private data
   memset(&gromit_data, 0, sizeof(gromit_data));

   id = usb_string_id(cdev);
   if (id < 0)
   {
      return id;
   }

   device_str[STRING_MANUFACTURER_IDX].id = id;
   device_desc.iManufacturer = id;
   device_str[STRING_MANUFACTURER_IDX].s = manufacturer;

   id = usb_string_id(cdev);
   if (id < 0)
   {
      return id;
   }

   device_str[STRING_PRODUCT_IDX].id = id;
   device_desc.iProduct = id;
   device_str[STRING_PRODUCT_IDX].s = longname;

   id = usb_string_id(cdev);
   if (id < 0)
   {
      return id;
   }

   device_str[STRING_SERIAL_IDX].id = id;
   device_desc.iSerialNumber = id;
   device_str[STRING_SERIAL_IDX].s = iSerialNumber;

   gromit_add_devices(cdev);

   gcnum = 0x0; /*usb_gadget_controller_number(gadget);*/
   PDEBUG (" USB Gadget Controller Number 0x%x\n", gcnum);
   PDEBUG ("Gadget driver name '%s', for gromit\n", gadget->name);
   if (gcnum >= 0)
   {
      device_desc.bcdDevice = cpu_to_le16(0x0200 + gcnum);
   }
   else 
   {
      /* gadget zero is so simple (for now, no altsettings) that
       * it SHOULD NOT have problems with bulk-capable hardware.
       * so just warn about unrcognized controllers -- don't panic.
       *
       * things like configuration and altsetting numbering
       * can need hardware-specific attention though.
       */
      pr_warning("%s: controller '%s' not recognized\n",
                 longname, gadget->name);
      device_desc.bcdDevice = cpu_to_le16(0x9999);
   }
   INFO(cdev, "%s, version: " DRIVER_VERSION "\n", longname);

   // Add attributes file to display request usage
   device_create_file(&gadget->dev, &dev_attr_req_stats);

   // Flag device as suspended by default
   cdev->suspended = 1;

   PDEBUG ("Manu: %s\n", manufacturer);

   return 0;
}

/*-------------------------------------------------------------------------*/
static void delete_gromit_device(struct gromit_private_data *data,
                                 struct gromit_devfile *file)
{
   PDEBUG("file=%p, id=%08X\n", file, file->dev->devt);

   if (file->dev != NULL)
   {
      // Delete file from system
      cdev_del(&file->cdev);
      device_destroy(data->class, file->dev->devt);
   }

   PDEBUG("Removed device %p\n", file);
}

/*-------------------------------------------------------------------------*/
static int gromit_comp_driver_unbind(struct usb_composite_dev *cdev)
{
   struct usb_gadget *gadget = cdev->gadget;

   PDEBUG ("Gromit driver unbind\n");

   // N.B. By the time we get here, the USB configuration and
   // functions have been shut down.

   // Remove stats file from sysfs
   device_remove_file(&gadget->dev, &dev_attr_req_stats);

   // Destroy device files and device class 
   PDEBUG("Destroying DATA\n");
   delete_gromit_device(&gromit_data, &gromit_data.data_dev);

   PDEBUG("Destroying CTRL\n");
   delete_gromit_device(&gromit_data, &gromit_data.ctrl_dev);

   PDEBUG ("Removed devices\n");

   if (gromit_data.class != NULL)
   {
      class_destroy(gromit_data.class);
   }

   PDEBUG ("Removed class\n");

   return 0;
}

/*-------------------------------------------------------------------------*/
static __refdata struct usb_composite_driver gromit_comp_driver = 
{
   .name    = "gromit_usb",
   .max_speed = USB_SPEED_FULL,
   .dev     = &device_desc,
   .strings = device_str_tablelist,
   .bind    = gromit_comp_driver_bind,
   .unbind  = gromit_comp_driver_unbind,
   .suspend = gromit_comp_driver_suspend,
   .resume  = gromit_comp_driver_resume,
};

static int __init init(void)
{
   PDEBUG ("GROMIT INIT\n");
   return usb_composite_probe(&gromit_comp_driver);
}
module_init(init);

static void __exit cleanup(void)
{
   usb_composite_unregister(&gromit_comp_driver);
}
module_exit(cleanup);
