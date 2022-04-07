# LISST-ABS-ros
Driver for the Sequoia LISST-ABS Acoustic Backscatter Sensor(SN6293)

## Maintainer
For all questions please reach to Jasleen Dhanoa (jkdhanoa@seas.upenn.edu).
The user manuals and detailed information about calibration and related papers are present on the ScalAR lab Google Drive folder: `Turbidity Sensor (LISST-ABS)`

## Installation
### Install this directory
Install this directory by using the following command:
```
git clone git@github.com:hsiehScalAR/LISTT-ABS-ros.git
```
### Build the ROS Workspace
The given commands are for running the driver in isolation.
```
$ mkdir catkin_ws
$ cd catkin_ws
$ mkdir src
$ cp -R path_where_directory_was_installed\LISST-ABS-ros\lisst_abs_sensor\ src\
$ source /opt/ros/ros-version-eg neotic/setup.bash 
$ catkin_make
$ source devel/setup.bash
```
### Plug in the LISST-ABS sensor
LISST-ABS is an Acoustic Backscatter Sensor which measures Suspended Sediment Concentration (in mg/L). The sensor is factory calibrated for particles in the range 75-90μm. Thus, it needs to be calibrated to determine a calibration factor against a sample solution of known calibration before employing it. 

The sensor communicates serially via RS232. It can be given power via USB-RS232 connection and a green LED light will start to blink. The LISST-ABS uses a baud rate of 9600 with 8 data bits, one stop bit, no parity, and no flow control. It outputs the uncalibrated concentration value and thus needs to be multiplied by the calibration factor to make use. The values are output once per second in scientific notation and are terminated with linefeed character.

## Code Organisation 

#### Config/Parameter File
The configuration is stored in `config/sensor_config.yaml`
```
device: /dev/ttyUSB0
baudrate: 9600
```
You might need to change the port the sensor is connected to. However, the default settings should work most of the times. You can find the port to which the sensor is connected to via `dmesg|tail`

#### Message Format
`msg/Turbidity.msg` file contains information about the message type published on the rostopic: `TurbidityData`. Currently the message format is :
```
Header header
float64 turbidity_mmt
```
A snippet of the message looks like this:
```
# =======================================
# topic:           TurbidityData
# msg_count:       11
# timestamp (sec): 1648653583.694172382
# - - -
header: 
  seq: 11
  stamp: 
    secs: 1648653582
    nsecs: 702092170
  frame_id: "Turbidity Levels"
turbidity_mmt: 1.21
```

#### Script
The script lisst_abs_sensor_publisher.py in scripts publishes the sensor data on the topic:'TurbidityData'. It opens a serial connection with the device listed in Parameter at the listed Baudrate with 8 data bits, one stop bit, no parity, and no flow control. It skips the first message as it contains the null character '\x00'. After obtaining the data it publishes the data on the rostopic: TurbidityData in the format specified in the msg file.

#### Launch File
The Launch File contains the package name: `lisst_abs_sensor`, the script name: `lisst_abs_sensor_publisher.py` and information on where to find the parameters: `/config/sensor_config.yaml` so that it can automatically launch the ROS driver by calling the launch file.

## Running the Driver
In order to run the driver ensure that the sensor is plugged into the computer and the green LED is blinking. Then run the driver by uisng the roslaunch command and the launch file.
```
$ cd catkin_ws
$ source devel\setup.bash
$ roslaunch lisst_abs_sensor lisst_abs.launch
```

## Recording and viewing the data in .bag file
In another terminal run the following command:
```
rosbag record -0 \path\bagfilename.bag \TurbidityData
```
After you have finished recording the data, you can view the recorded messages by using the following command:
```
time ros_readbagfile \path\bagfilename.bag
```

## Plotting real time data
You can also plot the data from the sensor in a real time plot by running rqt_plot in parallel terminal. Make sure you are subscribed to the rostopic: `\TurbidityData` and are plotting `\TurbidityData\turbidity_mmt values`.