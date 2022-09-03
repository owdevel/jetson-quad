# Jetson Quad

Main repository for orblam3 based jetson tracking


## Installation Instructions
1. Ensure there is a large amount of swap space, it is suggested to mount a swapfile of a least 10GB+ (tested with 20GB swapfile)
2. Clone this repo onto jetson
3. Enable and update submodules with `git submodule init` and `git submodule update`
4. Navigate to `src/orbslam3-ros/ORB_SLAM3`
5. Compile ORB SLAM3 with `./build.sh`, note that this process can take a couple hours on the Jetson TX2.
6. Navigate back to the root and run `catkin build`

