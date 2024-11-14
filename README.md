# T265_autoreconnect
This ROS program connects to the T265 camera and publishes a synchronized stream of 6DoF-pose, 2 fisheye images, gyroscope raw data, and most notably **tracker confidence**.
The program will also detect a broken pipe and attempt a reconnect, instead of just loosing all the data without a word. 
This ROS wrapper is based on a *patched* version of librealsense 2.50. The patch is described in this [issue](https://github.com/IntelRealSense/librealsense/issues/9030#issuecomment-962223017).

## Installation
1. Remove any existing librealsense2 packages from your system. In the following, we will compile a patched version of librealsense2 for ourselfes.
   ```console
   sudo apt remove *librealsense*
   ```  
2. Get the [patched version of librealsense2](https://github.com/fallow24/librealsense-2.50-patched).
3. Install the patch following these instructions:
### Installing the patch
1. ```console
   sudo apt-get update && sudo apt-get upgrade && sudo apt-get dist-upgrade
   sudo apt-get install libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev
   sudo apt-get install git wget cmake build-essential
   sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at
   sudo apt-get install v4l2ucp v4l2loopback-dkms
   ```
2. Download the [patch](https://github.com/fallow24/librealsense-2.50-patched/archive/refs/heads/master.zip) and go into the folder
3. Run the udev script from the **librealsense2 root directory**:
   ```console
   ./scripts/setup_udev_rules.sh
   ```
4. Run the kernel update script. *If a timeout occurs it can probably be ignored.* For further informtion visit the [official docs](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)
   ```console
   ./scripts/patch-realsense-ubuntu-lts.sh
   ```
5. Build the library, again start from the **librealsense2 root directory**
   ```console
   mkdir build && cd build
   cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=false -DBUILD_GRAPHICAL_EXAMPLES=false ../
   ```
   Note that I like to keep the binaries minimal. For further information on compilation flags visit the [official docs](https://dev.intelrealsense.com/docs/build-configuration).
   Finally, compile and install the library:
   ```console
   make
   sudo make install
   ```
   Or, if you want to uninstall:
   ```console
   sudo make uninstall
   ```
### Installing this ROS wrapper (auto_reconnect node)
Clone this repository into your catkin workspace
```console
cd ~/catkin_ws/src
git clone https://github.com/fallow24/T265_autoreconnect.git
cd ..
catkin_make
```
### Running the program
As of now, the program has no configuration parameters.
It will output pose data at 200 Hz and fisheye images at 30 Hz.
It will also perform an automatic reconnect if the pipeline gets broken, i.e., data streaming stops (happens sometimes on many systems).
And most notably, it outputs the internal "tacker confidence" estimation, that is usually hidden. 
```console
rosrun realsense_pipeline_fix auto_reconnect
```
   
