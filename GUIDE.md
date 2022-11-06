# Jetson Quad OS Guide

## Quickstart
### Turning the Jetson On
There is a three buttons on the carrier board.
Once power is supplied a light will turn on, to turn on the Jetson you have to press the button closest to the power light.
If successful, a second light should come on in between the power light and the power button.

### Login Credentials
- Host: jetson-quad
- User: jetson
- Password: quad

### There's no GUI!
```bash
sudo init 5
```


## Hardware
 - [NVidia Jetson TX2 4GB](https://developer.nvidia.com/embedded/jetson-tx2-4gb)
 - [Orbitty Carrier for NVIDIA Jetson TX2/TX2i](https://connecttech.com/product/orbitty-carrier-for-nvidia-jetson-tx2-tx1/)
 - [Hex Cube Black Flight Controller](https://docs.px4.io/main/en/flight_controller/pixhawk-2.html) (formerly Pixhawk 2.1)

### Forced Recovery Mode
Needed to flash the orbitty. The button order is POWER, RESET, RECOVERY with power being closest to the LEDs.
1. Ensure the system is on, if not press and release POWER, the SYS light should come on (2 total lights).
2. Press and hold RECOVERY 
3. Press and release RESET
4. Wait for around 2s then release RECOVERY
5. Running `lsusb` on a host connected via the micro-usb slot should yield a NVidia Corp. device.

## Base OS - NVIDIA Jetpack
Based on Ubuntu 18.04, custom NVIDIA version utilising the board support package for the Orbitty Carrier Board (CTI-L4T).
Jetpack 4.6 is known to work, later versions are buggy.

### Installation
The installation process requires a host computer running Ubuntu 18.04 as well as ~25GB of extra storage on top of the base OS.
The carrier board micro-usb port needs to be connected to the host computer, and the 3 buttons needed for Forced Recovery Mode to be accessible.
A hdmi display, keyboard and mouse are also required to finish the setup process.

1. Install the BSP using the guide from Connect Tech - [KDB373](https://connecttech.com/resource-center/kdb373/)
    - For recovery mode see above section
    - Once the initial flash is complete, the micro-usb cable can be unplugged as long as the board is still receiving power. If is recommended to disconnect the board to avoid driver errors.
    - If booting from the SD card was enabled previously, the process has to be reversed or the SD card removed in order to boot a fresh system.
    - The initial setup is very buggy, be prepared to hit the reset button if it freezes for too long.
    - The GUI may fail to load and you may get stuck on a terminal-like log screen, use `Ctrl+Alt+F2-F6` to access different TTYs
    - You may need to use an ethernet cable to continue

2. Enable Running from SD Card (see below, Optional but highly recommended)

2. Install the SDK components using the guide from Connect Tech - [KDB374](https://connecttech.com/resource-center/kdb374/)
    - Whilst technically optional, these components are specifically accelerated for this architecture and throughly recommended.
    - These do not fit on internal storage and needs to be run off external memory.
    - Note that by default this installs OpenCV 3.2 as of writing this

### Running from SD Card
Default internal flash is very limited, any non-production work should be run off of a SD card for storage.
[Source](https://www.jetsonhacks.com/2017/01/26/run-jetson-tx1-sd-card/)

1. Format SD Card (GPT, ext4)
2. Figure out which partition is the sd card with `lsblk`, should be `/dev/mmcblk2p1`
3. Mount SD Card (Click GUI or `sudo mount`)
4. Copy root dir to SD Card `sudo cp -avx / /media/jetson/jetson-sd`
5. Make a backup of `/media/jetson/jetson-sd/boot/extlinux/extlinux.conf`
6. Edit `boot/extlinux/extlinux.conf`, replacing `root=/dev/mmcblk0p1` with the device in step 2
7. Reboot (TX2 will default to boot from sd card if available)
8. Check the correct partition is mounted with `lsblk` or `df -h`


### GUI/Headless
Run the Jetson in Headless mode - [Source](https://www.forecr.io/blogs/bsp-development/how-to-disable-desktop-gui-on-jetson-modules)
#### Temporary Disable/Enable

- Enable `sudo init 5`
- Disable `sudo init 3`

#### Permanent Disable/Enable

- Enable `sudo systemctl set-default graphical.target`
- Disable `sudo systemctl set-default multi-user.target`

### Swap Space
[temporary swapfile](https://linuxize.com/post/how-to-add-swap-space-on-ubuntu-20-04/)

```bash
sudo fallocate -l 20G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

## Hardware 
### Wifi Dongle
#### Driver
The WiFi dongle is a TL-WN823N_V2 with a rtl8192eu chip.
The drivers in the Ubuntu18.04 kernel can be unstable and extremely slow at times, and the official drivers are not properly architectured for ARM.

Due to using a popular chip internally, a generic driver for that chip is available for install.
The driver is mirrored here on [github](https://github.com/Mange/rtl8192eu-linux-driver) with installation instructions. 
Note the step to change the makefile platform configuration to AARCH64 as the default is for x86 pcs.

#### Disable Power Save Mode
The WiFi chip will periodically enter power save mode by default which adds latency to the wifi connection.
Follow this post to disable this functionality.
[Jetson Disable Wifi Power Management](https://github.com/robwaat/Tutorial/blob/master/Jetson%20Disable%20Wifi%20Power%20Management.md)

#### Connect on Boot
By default, wifi connections in Ubuntu is managed per user, and as such if a user does not log in, no wifi will connect.
This complicated headless usage however.

1. Login and connect to a wifi network through the GUI
2. Open the network file in an editor from `/etc/NetworkManager/system-connections/`
3. Comment out the line with `permission=` (add a `#`)

[Source](https://askubuntu.com/questions/16376/connect-to-network-before-user-login)


### Realsense D435I
The realsense drivers packaged in the apt repository do not use CUDA and also utilise a software usb driver which is slower and less stable.
Compiling the realsense drivers from [source](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_jetson.md) gives us better performance.

1. Ensure the repository driver is not installed
2. Clone the librealsense repo and checkout appropriate version (realsense-ros v2.3.2 requires librealsense v2.50.0 at time of writing)
3. Follow the build steps from [source](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_jetson.md) 
4. Disable installation from repositories with `sudo apt-mark hold ros-melodic-librealsense2`
4. (Optional) If realsense repositories are enabled, disable installation from repositories with `sudo apt-mark hold ros-melodic-librealsense2`

NOTE: Sometimes rosdep will override if using `rosdep install` so pay attention that librealsense2 doesn't get installed.

## Software

### ROS
ROS melodic is the appropriate version for this distro.
The [standard install](http://wiki.ros.org/melodic/Installation/Ubuntu) path works, given the embedded nature and running headless, the ROS-Base package can be used as no GUI is needed.

### MAVROS
For MAVROS to run correctly over serial, its necessary

1. `sudo adduser jetson dialout`
1. MAVROS setup - navigate and run `./src/mavros/mavros/scripts/install_geographiclibs_datasets.sh`

### Jetson-quad repo

Repo setup instructions. Replace `jetson-quad/` with the path to the cloned folder.

IMPORTANT: Read the subsections of the rosdep install, if using the self-compiled version of realsense driver it can accidentally get overriden causing driver issues due to multiple conflicting entries which are hard to solve.

1. Clone with `git clone --recurse <link>` as git submodules are in use
2. Navigate to `jetson-quad/src/orbslam3-ros/ORBSLAM/` and run `build.sh` to build ORBSLAM3 (note this can take a couple hours, ensure that there is extra swap space allocated)
3. `sudo ln -s /usr/include/opencv4 /usr/include/opencv` - opencv is in a non-standard location on the TX2, symbolic link the directories to avoid errors
4. Install missing packages with rosdep - `rosdep install --from-paths jetson-quad/src --ignore-src -r`
    - WARNING: This process will error on the python package `simplejpeg` for ROSBoard installed in the next step
    - WARNING: This process will install the `ros-melodic-librealsense2` driver by default.
    If using the self-compiled driver, it's better to run with the `-s` flag to simulate instead of the `-r` flag, this will list all the install commands to be run manually.
5. ROSBoard Setup (Optional)
    - `sudo -H pip3 install simplejpeg rospkg` - install python3 dependencies
    - `pip3 install tornado` - for some reason tornado fails to function properly on the system-wide packages, so we install it for the user python install
6. `catkin build` - build everything, extra swap space is recommended, shouldn't take longer than 15min


#### Running Nodes

##### ROSBoard
1. `rosrun rosboard rosboard_node`
2. Connect via `http://ip-of-quad:8888/`

##### CPU Monitor
1. `roslaunch cpu_monitor cpu_monitor.launch`

##### MAVLink
1. `roslaunch px4-imu px4.launch`
Enable imu and other streamed data
2. `rosrun px4-imu listener.py`

##### Camera Node
1. `roslaunch orbslam3_ros camera.launch`

##### ORBSLAM Node
Note that the launch files launch in GUI mode by default, to launch without gui append `gui:=false` to the command.
- Mono - `roslaunch orbslam3_ros orbslam_mono.launch`
- RGBD - `roslaunch orbslam3_ros orbslam.launch`
- RGBDi - `roslaunch orbslam3_ros orbslami.launch`