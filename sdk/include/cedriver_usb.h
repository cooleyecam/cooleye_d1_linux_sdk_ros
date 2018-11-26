#ifndef CEDRIVER_USB_H
#define CEDRIVER_USB_H

#include "cedriver_global_config.h"
#include <libusb-1.0/libusb.h>

struct ce_usb_dev_stru
{
    libusb_device *dev;
    libusb_device_handle *handle;
    unsigned short vid;
    unsigned short pid;         /* Product ID */
    unsigned char is_open;      /* When device is opened, val = 1 */
    unsigned char busnum;       /* The bus number of this device */
    unsigned char devaddr;      /* The device address*/
    unsigned char filler;       /* Padding to make struct = 16 bytes */
};

int ce_usb_open(void);
void ce_usb_close(void);

libusb_device_handle *ce_usb_gethandle(int index);

#endif
