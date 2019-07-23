# Welcome to Home of Zumo Pi
![zumopi_portrait](https://github.com/linZHank/zumo_pi/blob/master/images/zumo_pi_art.jpg)

## Major Hardware
[Raspberry Pi 3 Model B+](https://www.raspberrypi.org/products/raspberry-pi-3-model-b-plus/)

[Adafruit DC and Stepper Motor HAT for Raspberry Pi](https://learn.adafruit.com/adafruit-dc-and-stepper-motor-hat-for-raspberry-pi/overview)

[Zumo Chassis Kit](https://www.pololu.com/product/1418)

[100:1 Micro Metal Gearmotor HP 6V](https://www.pololu.com/product/1101)

## Install
1. Flash [Ubuntu 16.04 (LXDE) and pre-installed ROS-Kinetic image](https://downloads.ubiquityrobotics.com/) into your micro ssd.
2. Install [Adafruit-Motor-HAT-Python-Library](https://github.com/adafruit/Adafruit-Motor-HAT-Python-Library)
```console
cd ~
git clone https://github.com/adafruit/Adafruit-Motor-HAT-Python-Library.git
cd Adafruit-Motor-HAT-Python-Library
python setup.py install
```
3. Install [Catkin Command Line Tools](https://catkin-tools.readthedocs.io/en/latest/)
4. Create ROS workspace and build this package
```console
cd ~
mkdir -p ros_ws/src
cd ros_ws/src
git clone https://github.com/linZHank/zumo_pi.git
cd ~/ros_ws
catkin build
```
5. Install [teleop_twist_keyboard](http://wiki.ros.org/teleop_twist_keyboard): `sudo apt install ros-kinetic-teleop-twist-keyboard`

## Have Fun
1. Open a terminal, then `roslaunch zumo_pi zumo_driver.launch`
2. Open a new terminal, then `rosrun teleop_twist_keyboard teleop_twist_keyboard.py`
