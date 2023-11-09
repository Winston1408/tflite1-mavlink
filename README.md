# tflite1-mavlink
Image recognition and tracking based on TensorFlow-Lite-Object-Detection-on-Android-and-Raspberry-Pi and control a gimbal to track the recognized object at the center of image.  Gimbal based on PCA9685.  Also send message on MAVLink.

To run this code:
python3 TFLite_detection_webcam.py --modeldir=Sample_TFLite_model --edgetpu

Required hardware:
1)  Raspberry Pi 4 Computer Model B 8GB Single Board Computer Suitable for Building Mini PC/Smart Robot/Game Console/Workstation/Media Center/Etc.
2)  Coral USB Accelerator
3)  Raspberry Pi Camera Module V2-8 Megapixel,1080p (RPI-CAM-V2)
4)  Pixhawk PX4 PIX 2.4.8 32 Bit Flight Controller Autopilot with 4G SD Safety Switch Buzzer PPM I2C for RC Quadcopter Ardupilot
5)  Arducam Upgraded Camera Pan Tilt Platform Compatible with Raspberry Pi Camera Module 3/V1/V2, Nvidia Jetson Nano/Xavier NX

Required software packages:
1)  Install Rasbian Headless (32-bit Bullseye) on RasPi4B
2)  Install TensorFlow-Lite-Object-Detection from Google Coral
3)  Install ArduCam
4)  Install MavLink
To install these packages, simply run:
  $ sh get_pi_requirements.sh in the package.

References:
1)  https://www.tomshardware.com/reviews/raspberry-pi-headless-setup-how-to,6028.html
2)  https://github.com/EdjeElectronics/TensorFlow-Lite-Object-Detection-on-Android-and-Raspberry-Pi/blob/master/deploy_guides/Raspberry_Pi_Guide.md
3)  https://github.com/ArduCAM/PCA9685
4)  https://ardupilot.org/dev/docs/raspberry-pi-via-mavlink.html

Raspberry Pi4 HW Configuraton 
1)  Set up RPi for UART comm in raspi-config.
    In "Interfaces" tab, Disable Serial Console.  
    Enable for Serial Port.
2)  Enable camera
    sudo raspi-config
    Then select: 3 Interface Options    Configure connections to peripherals.
    Then select: I1 Legacy Camera Enable/disable legacy camera support and follow the directions to enable and then reboot.
3)  In /boot/config.txt append
    dtoverlay=disable-bt
    enable_uart=1
4)  In /etc/pip.conf append
    break-system-packages = true

Download this code and run the shell script:
git clone https://github.com/winston1408/tflite1-mavlink.git
cd tflite1-mavlink/
sh get_pi_requirements.sh
