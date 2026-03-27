
UART2: TX GPIO4, RX GPIO5
UART3: TX GPIO8, RX GPIO9
UART4: TX GPIO12, RX GPIO13

Boot config file needs to be changed on Raspberry pi 5:

$ sudo nano /boot/firmware/config.txt

At the very end of the file add the lines:
# UART configuration
dtoverlay=uart2
dtoverlay=uart3
dtoverlay=uart4

$ sudo reboot

To check which pins on the J8 header are configured as UART run the following command:
$ pinctrl

To find out which device is active run the following command:
$ cd /dev/
$ ls

The UART interfaces are usually called "ttyAMAx". So UART2 would be "ttyAMA2".

