# Install ROSwan
## On Beaglebone
Run:
```
cd ~/setup/
sh setup_roswan.sh
```
## On remote computer
Run:
```
sh setup_roswan_com.sh
```
After that remember to run the following command to configure the bash environment before running ROSwan
```
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://<BEAGLEBONE IP>:11311
export ROS_HOSTNAME=<REMOTE COMPUTER IP>
```
You can also put these three lines at the end of `~/.bashrc` for convenience.
