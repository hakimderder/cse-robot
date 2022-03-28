# Create wireless network on Raspberry Pi 4

Follow the tutorial on th website of Raspberry Pi :
- [setting-up-a-routed-wireless-access-point](https://www.raspberrypi.org/documentation/computers/configuration.html#setting-up-a-routed-wireless-access-point)


## SD card & boot details

Use Raspberry Pi Imager to create your bootable sd card :
- [Raspberry Imager](https://downloads.raspberrypi.org/imager/imager_latest_amd64.deb)


Add the following line to the boot/config.txt file :
`enable_uart=1`

For help, try : 
- [setting-up-your-raspberry-pi](https://www.raspberrypi.org/documentation/computers/getting-started.html#setting-up-your-raspberry-pi)


## Connect to your Rpi using uart


Connect your Raspberry Pi with a USB <-> UART câble, like this :
![uart_rpi](./../images/rpi4_uart_connection.png)


