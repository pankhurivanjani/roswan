# Setup Beaglebone
1. Plug in your SD card to the beaglebone.
2. It will be best for the beaglebone to have Ethernet connection at this step.  **Plug in the Ethernet cable to your computer before booting up**. Due to a small bug, it will take a pretty long time for the beaglebone without the Ethernet cable plugged in dhcp configuration.  For more details please refer to the original discussion:  https://groups.google.com/forum/#!topic/beagleboard/-5EAWKMTqsg.
3. Plug in the power to  boot up the beaglebone.
4. Use ssh to access the beaglebone.
5. Copy the setup file over to the beaglebone manually or run:
```
cd ~
wget https://github.com/subnero1/roswan/raw/develop/BBB/setup.tar.gz
tar -xzvf setup.tar.gz
```
6. First, run the shell script to downgrade the linux kernel to support the capemanager for easy gpio control.
```
cd ~/setup/
sh downgrade_kernel.sh 
```
Remember to reboot

1. Run the shell script to setup wifi, udev and  gpio.
```
cd ~/setup/
sh setup_BBB.sh
```
Remember to reboot.

*For more detailed information please refer to the* [original swan provision](https://github.com/subnero1/swan/blob/master/sw/provisioning/master/setup-master.md).
