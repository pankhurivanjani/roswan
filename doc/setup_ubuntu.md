# Flashing Ubuntu 16.04 to Beaglebone Black (on Ubuntu 16.04
1. Download `setup.tar.gz` on to your computer and extract it manually or run:
```
cd ~
wget https://github.com/subnero1/roswan/raw/develop/BBB/setup.tar.gz
tar -xzvf setup.tar.gz
```
2. Download the ubuntu 16.04 beaglebone compressed image  and  put it inside the `setup` folder manually or run:
```
cd ~/setup/
wget https://rcn-ee.com/rootfs/2018-03-09/elinux/ubuntu-16.04.4-console-armhf-2018-03-09.tar.xz
```
4. Navigate to the folder and run the command below to flash your SD card.
```
cd ~/setup/
sh setup_ubuntu.sh
```
*Please follow the instructions from the terminal to insert your SD card and key in you SD mounted path.*

*For more detailed information please refer to*  https://elinux.org/BeagleBoardUbuntu.