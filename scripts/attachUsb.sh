#!/bin/bash

usbipd.exe wsl list

echo "Attempting to attach..."
usbipd.exe wsl attach -d ubuntu -b 6-3

usbipd.exe wsl list

echo "Grant permissions to access device."
sudo chmod a+rw /dev/ttyACM0

ls -l /dev/ttyACM0