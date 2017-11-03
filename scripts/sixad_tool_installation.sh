#!/bin/bash

GREEN='\e[92m'
END='\e[0m'
sudo apt-get install bluetooth bluez-tools bluez bluez-hcidump checkinstall libusb-dev libbluetooth-dev joystick pyqt4-dev-tools libjack0 libjack0-dev
                     pi-bluetooth

echo -e "${GREEN} Installing sixpair  ${END}"
wget http://www.pabr.org/sixlinux/sixpair.c -P ~/.
gcc -o sixpair sixpair.c -lusb


echo -e "${GREEN} Installing sixad tool  ${END}"
wget http://sourceforge.net/projects/qtsixa/files/QtSixA%201.5.1/QtSixA-1.5.1-src.tar.gz -P ~/.
tar xfvz ~/QtSixA-1.5.1-src.tar.gz  -C ~/.
cd ~/QtSixA-1.5.1/sixad
make
sudo mkdir -p /var/lib/sixad/profiles
sudo checkinstall

echo -e "${GREEN} End  ${END}"
echo -e "${GREEN} Last step sudo vim /etc/default/sixad - change LEGACY=0 to LEGACY=2  ${END}"
echo -e "${GREEN} If something fail, check this web: https://www.raspberrypi.org/forums/viewtopic.php?f=28&t=137018  ${END}"
