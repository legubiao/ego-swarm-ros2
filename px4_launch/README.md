# Launch PX4 ROS2 Gazebo simulation

## Install PX4

### Setup Micro XRCE-DDS Agent & Client
```bash
# choose a location to place the Micro XRCE-DDS Agent source code.
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

### use instruction 
run single.sh first, only run multiple.sh after the single initialize finished.