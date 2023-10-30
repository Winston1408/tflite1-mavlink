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
1)  Install Rasbian Headless on RasPi4B
2)  Install TensorFlow-Lite-Object-Detection
3)  Install ArduCam
4)  Install MavLink
To install these packages, simply run:
  >> sh get_pi_requirements.sh

References:
1)  https://github.com/EdjeElectronics/TensorFlow-Lite-Object-Detection-on-Android-and-Raspberry-Pi/blob/master/deploy_guides/Raspberry_Pi_Guide.md
2)  https://github.com/ArduCAM/PCA9685
3)  https://ardupilot.org/dev/docs/raspberry-pi-via-mavlink.html
