#!/bin/bash

#Set up RPi for UART comm in raspi-config.  Disable UART for console.  Enable for serial port hardware
#in /boot/config.txt add "dtover=disable-bt"
#if ttyAMA0 isn't in /dev, enable_uart=1 in /boot/config

# Install Python3
sudo apt-get update
sudo apt-get -y upgrade
sudo apt-get -y install python3 python3-pip python3-dev python3-opencv 
sudo apt-get -y install python3-wxgtk4.0 python3-matplotlib python3-lxml 
sudo apt-get -y install python3-pygame python3-scipy

sudo pip install future
sudo apt-get install screen


sudo apt-get -y install build-essential cmake pkg-config
sudo apt-get -y install libjpeg-dev libtiff5-dev libjasper-dev libpng-dev
sudo apt-get -y install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev
sudo apt-get -y install libxvidcore-dev libx264-dev
sudo apt-get -y install libgtk2.0-dev libgtk-3-dev
sudo apt-get -y install libatlas-base-dev gfortran

sudo apt-get -y install qt4-dev-tools 

# Get packages required for OpenCV

sudo python3 -m pip install -U pip
sudo python3 -m pip install -U matplotlib
sudo apt-get -y install libqt5gui5
sudo modprobe bcm2835-v4l2
sudo apt-get -y install libqt5test5

# Need to get an older version of OpenCV because version 4 has errors
pip3 install opencv-python==3.4.11.41

# Dronekit and MAVLink
sudo pip install pyserial
sudo pip install dronekit
sudo pip install MAVProxy
git clone https://github.com/mavlink/mavlink.git --recursive
git clone https://github.com/pymavlink.git recursive
python3 -m pip install -r pymavlink/requirements.txt

pip3 install PyYAML mavproxy --user
pip3 install mavproxy pymavlink --user --upgrade

export PATH="/usr/local/bin:$PATH"
source ~/.bash_profile
echo $PATH

echo 'export PATH="$PATH:$HOME/.local/bin"' >> ~/.bashrc

# Get packages required for TensorFlow
# Using the tflite_runtime packages available at https://www.tensorflow.org/lite/guide/python
# Will change to just 'pip3 install tensorflow' once newer versions of TF are added to piwheels

#pip3 install tensorflow

version=$(python3 -c 'import sys; print(".".join(map(str, sys.version_info[:2])))')
echo $version

pip3 install https://github.com/google-coral/pycoral/releases/download/v2.0.0/tflite_runtime-2.5.0.post1-cp39-cp39-linux_armv7l.whl
