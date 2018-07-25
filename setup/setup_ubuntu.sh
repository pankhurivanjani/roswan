#!/bin/bash -e

FILE="ubuntu-16.04.4-console-armhf-2018-03-09.tar.xz"

if [ -f $FILE ]; then
   echo "File $FILE exists."
else
   echo "File $FILE does not exist."
   wget https://rcn-ee.com/rootfs/2018-03-09/elinux/ubuntu-16.04.4-console-armhf-2018-03-09.tar.xz
fi

echo "extracting $FILE ..."
tar xf ubuntu-16.04.4-console-armhf-2018-03-09.tar.xz
cd ubuntu-16.04.4-console-armhf-2018-03-09/
read -p "Have you inserted your SD card? [Press Enter]:" yn
sudo ./setup_sdcard.sh --probe-mmc
read -p "SD location? e.g. [/dev/mmcblk0]: " SD_PATH
sudo ./setup_sdcard.sh --mmc $SD_PATH --dtb beaglebone
rm -rf ubuntu-16.04.4-console-armhf-2018-03-09
exit 0
