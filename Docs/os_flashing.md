# Getting Start with Ubiquity Pi Image

## Download Image
Download the latest [Pi image](https://downloads.ubiquityrobotics.com/pi.html) with pre-installed ROS-Kinetic. I downloaded [this image](https://ubiquity-pi-image.sfo2.cdn.digitaloceanspaces.com/2019-06-19-ubiquity-xenial-lxde-raspberry-pi.img.xz), saved at `~/Downloads/2019-06-19-ubiquity-xenial-lxde-raspberry-pi.img.xz`.

## Flash Image
There are at least three tools available: 1 and 2 are for Linux user only, 3 works on any OS.
### 1. Command line
```console
sudo apt-get install gddrescue xz-utils
sudo ddrescue -D --force 2019-06-19-ubiquity-xenial-lxde-raspberry-pi.img.xz /dev/sdc
```
### 2. Graphical tool
> Install [gnome-disk-utility](https://gitlab.gnome.org/GNOME/gnome-disk-utility/), `$ sudo apt-get install gnome-disk-utility` if you don't have one.

Please refer to [this video](https://youtu.be/_woWvmHl3Rc) to do so.
### 3. [Etcher](https://www.balena.io/etcher/)
`Select image`, `Select drive`, `Flash!`

## Notes
- When the Raspberry Pi boots for the first time, it resizes the file system to fill the SD card, this can make the first boot take some time.
- You can connect Zumo with built-in Wifi access point on your Raspberry Pi. The SSID is `ubiquityrobotXXXX` where `XXXX` is part of the MAC address. The wifi password is `robotseverywhere`.
- To remote login to your Raspberry Pi (or Zumo) with `ssh ubuntu@10.42.0.1`, password is `ubuntu`. Make sure you have connected to `ubiquityrobotXXXX` (You don't have to disconnect other network connections). 
