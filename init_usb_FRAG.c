void init_usb() {

    int idVendor = 0x1c40;
    int idProduct = 0x05b7;

    struct usb_bus *busses;

    usb_init();
    usb_find_busses();
    usb_find_devices();

    busses = usb_get_busses();

    struct usb_bus *bus;
    int c, i, a;
    int ireturn;

    for (bus = busses; bus; bus = bus->next) {
        struct usb_device *dev;

        for (dev = bus->devices; dev; dev = dev->next) {
            /* Check if this device is a printer */
            printf("DEBUG: usb_reset(dev): %s: class %d, vendor:%d(%d), product:%d(%d)=========================\n",
                    dev->filename, dev->descriptor.bDeviceClass,
                    dev->descriptor.idVendor, idVendor,
                    dev->descriptor.idProduct, idProduct
                    );
            if ((dev->descriptor.idVendor == idVendor) && (dev->descriptor.idProduct == idProduct)) {
                /* Open the device, claim the interface and do your processing */
                fflush(stdout);
                usb_dev_handle *dh = usb_open(dev);
#ifdef __linux
                usb_detach_kernel_driver_np(dh, 0);
#endif
                if ((ireturn = usb_reset(dh)) < 0) {
                    printf("Failed\n");
                } else {
                    printf("Succeeded\n");
                }
                printf("DEBUG: usb_reset(dev): return %d: %s: class %d\n", ireturn, dev->filename, dev->descriptor.bDeviceClass);
                fflush(stdout);
            }

            /* Loop through all of the configurations */
            for (c = 0; c < dev->descriptor.bNumConfigurations; c++) {
                printf("DEBUG: usb_reset(dev): c %d, dev->descriptor.bNumConfigurations %d\n", c, dev->descriptor.bNumConfigurations);
                /* Loop through all of the interfaces */
                printf("DEBUG: usb_reset(dev): dev->config %ld\n", dev->config);
                if (dev->config <= 0)
                    continue;
                fflush(stdout);
                for (i = 0; i < dev->config[c].bNumInterfaces; i++) {
                    /* Loop through all of the alternate settings */
                    for (a = 0; a < dev->config[c].interface[i].num_altsetting; a++) {
                        /* Check if this interface is a printer */
                        printf("DEBUG: usb_reset(dev): %s: class %d, vendor:%d(%d), product:%d(%d)\n",
                                dev->filename, dev->descriptor.bDeviceClass,
                                dev->descriptor.idVendor, idVendor,
                                dev->descriptor.idProduct, idProduct
                                );
                        if ((dev->descriptor.idVendor == idVendor) && (dev->descriptor.idProduct == idProduct)) {
                            /* Open the device, set the alternate setting, claim the interface and do your processing */
                            printf("DEBUG: usb_reset(dev): dev %ld: dev->dev %ld\n", dev, dev->dev);
                            fflush(stdout);
                            usb_dev_handle *dh = usb_open(dev);
#ifdef __linux
                            usb_detach_kernel_driver_np(dh, 0);
#endif
                            if ((ireturn = usb_reset(dh)) < 0) {
                                printf("Failed\n");
                            } else {
                                printf("Succeeded\n");
                            }
                            printf("DEBUG: usb_reset(dev): return %d: %s: class %d\n", ireturn, dev->filename, dev->descriptor.bDeviceClass);
                            fflush(stdout);
                        }
                    }
                }
            }
        }
    }

}

