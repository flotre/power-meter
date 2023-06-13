#!/bin/bash

gcc bootsel_usb.c -o bootsel_usb -I/usr/include/libusb-1.0 -L/usr/lib/arm-linux-gnueabihf -lusb-1.0