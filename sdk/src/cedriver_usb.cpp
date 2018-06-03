#include <stdio.h>
#include <libusb-1.0/libusb.h>
#include "cedriver_usb.h"

static int ce_usb_camd1_num;
static struct  ce_usb_dev_stru ce_usb_dev[MAXDEVICES];
static libusb_device **libusb_device_list;

static int ce_usb_lookup_cam_handle(libusb_device *dev)
{
    struct libusb_device_descriptor desc;
    libusb_get_device_descriptor(dev, &desc);
    if ( (VID_CE_D1 == desc.idVendor) && (PID_CE_D1 == desc.idProduct) )
        return SUCCESS;
    return ERROR;
}

libusb_device_handle * ce_usb_gethandle(int index)
{
    return ce_usb_dev[index].handle;
}

int ce_usb_open(void)
{
    int r;

    r = libusb_init(NULL);
    if (r)
    {
        printf("celog:  init libusb Error !\n");
        return -1;
    }

    int cnt_dev = libusb_get_device_list(NULL, &libusb_device_list);
    if ( cnt_dev < 0 )
    {
        printf("celog: Error in enumerating devices !\n");
        return -2;
    }

    ce_usb_camd1_num = 0;

    for (int i = 0; i < cnt_dev; ++i )
    {
        libusb_device *tdev = libusb_device_list[i];
        if ( SUCCESS == ce_usb_lookup_cam_handle(tdev) )
        {
            ce_usb_dev[ce_usb_camd1_num].dev = tdev;
            r = libusb_open(tdev, &ce_usb_dev[ce_usb_camd1_num].handle);
            if ( r )
            {
                printf("celog: open usb camD1 fail , r=%d\n",r);
                return -3;
            }
            ++ce_usb_camd1_num;
        }
        if(ce_usb_camd1_num>MAXDEVICES)
            break;
    }
    return ce_usb_camd1_num;
}

void ce_usb_close(void)
{
    for (int i = 0; i < ce_usb_camd1_num; ++i )
    {
        libusb_close(ce_usb_dev[i].handle);
    }
    libusb_free_device_list(libusb_device_list, 1);
    libusb_exit(NULL);
}
