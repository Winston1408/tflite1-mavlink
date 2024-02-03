######## Webcam Object Detection Using Tensorflow-trained Classifier #########
######## Command arduplane over MAVLink ########
# To run:
# python3 TFLite_detection_webcam.py
#



from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil  # Needed for command message definitions
import time
import math
import os
import argparse
import cv2
import numpy as np
import sys
import time
from threading import Thread
import importlib.util

import adafruit_servokit

##################### TensorFlow ##########################
# Import TensorFlow libraries
# 'tflite_runtime' must be installed 
# from tflite_runtime import interpreter library and load_delegate library for Coral Edge TPU
pkg = importlib.util.find_spec('tflite_runtime')
from tflite_runtime.interpreter import Interpreter, load_delegate

##################### Camera Gimbal Control ##########################

class ServoKit(object):
    servo_ch_x = 1
    servo_ch_y = 0
    default_x_deg = 90 #goes from 0 to 180
    default_y_deg = 90 #goes from 0 to 180
    current_x_deg = default_x_deg 
    current_y_deg = default_y_deg
    pix_per_x_deg = 180 # used to adjust movement sensitivity
    pix_per_y_deg = 50 # used to adjust movement sensitivity
    max_x_deg = default_x_deg + 45
    min_x_deg = default_x_deg - 45
    max_y_deg = default_y_deg + 45
    min_y_deg = default_y_deg - 45    

    def __init__(self):
        print("Initializing the servo...")
        self.kit = adafruit_servokit.ServoKit(channels=16)
        self.num_ports = 2
        self.resetAll()
        self.setAngle(self.servo_ch_y, int(self.current_y_deg))
        self.setAngle(self.servo_ch_x, int(self.current_x_deg))   
        print("Initializing complete.")

    def setAngle(self, port, angle):
        if(port == self.servo_ch_y):
            if (angle < self.min_y_deg):
                self.kit.servo[port].angle = self.min_y_deg
            elif (angle > self.max_y_deg):
                self.kit.servo[port].angle = self.max_y_deg
            else:
                self.kit.servo[port].angle = angle
            print("servo set angle y: "+str(angle))
        if(port == self.servo_ch_x):
            if (angle < self.min_x_deg):
                self.kit.servo[port].angle = self.min_x_deg
            elif (angle > self.max_x_deg):
                self.kit.servo[port].angle = self.max_x_deg
            else:
                self.kit.servo[port].angle = angle
            print("servo set angle x: "+str(angle))

    ########### Calculate and control horizontally ##########
    def moveX(self,x_error):
        self.current_x_deg = self.current_x_deg + x_error/self.pix_per_x_deg
        if(self.current_x_deg < self.min_x_deg):
            self.current_x_deg = self.min_x_deg
        if(self.current_x_deg > self.max_x_deg):
            self.current_x_deg = self.max_x_deg    
        print("gimbal x degree: "+str(self.current_x_deg))
        # Command servo to change
        self.setAngle(self.servo_ch_x, int(self.current_x_deg))

    ########### Calculate and control vertically ##########
    def moveY(self,y_error):
        self.current_y_deg = self.current_y_deg + y_error/self.pix_per_y_deg
        if(self.current_y_deg < self.min_y_deg):
            self.current_y_deg = self.min_y_deg
        if(self.current_y_deg > self.max_y_deg):
            self.current_y_deg = self.max_y_deg                      
        print("gimbal y degree: "+str(self.current_y_deg))
        # Command servo to change
        self.setAngle(self.servo_ch_y, int(self.current_y_deg))

        
    def getAngle(self, port):
        if(port < 2):
            return self.kit.servo[port].angle

    def reset(self, port):
        if(port == 0):
            self.kit.servo[port].angle = self.default_y_deg
        if(port == 1):
            self.kit.servo[port].angle = self.default_x_deg

    def resetAll(self):
        self.kit.servo[self.servo_ch_y].angle = self.default_y_deg
        self.kit.servo[self.servo_ch_x].angle = self.default_x_deg
        print("Servo ch y: "+str(self.default_y_deg))
        print("Servo ch x: "+str(self.default_x_deg))        


##################### Drone Control-via Dronekit ##########################
class DroneKitControl(object):
    # Connect to the Vehicle
    conn_string = '/dev/ttyAMA0'
    servo_ch_elevator = 2
    servo_ch_rudder = 4
    pwm_at_90_x = 1600 # servo position when flying straight
    pwm_at_90_y = 1600 
    
    default_x_deg = 90 #goes from 0 to 180
    default_y_deg = 90 #goes from 0 to 180
    current_x_deg = default_x_deg 
    current_y_deg = default_y_deg
    pix_per_x_deg = 180 # used to adjust movement sensitivity
    pix_per_y_deg = 50 # used to adjust movement sensitivity
    max_x_deg = default_x_deg + 45
    min_x_deg = default_x_deg - 45
    max_y_deg = default_y_deg + 45
    min_y_deg = default_y_deg - 45    
    
    def __init__(self):
        print("Connecting to vehicle on: " + str(self.conn_string))  
        self.vehicle = connect(self.conn_string, wait_ready=True, baud=57600)
        print("Connected to vehicle on: " + str(self.conn_string))  
        self.disarm()
        time.sleep(3)  

    # Preflight/launch
    def disarm(self):
        self.vehicle.armed = False
        print("Vehicle disarmed")

    def arm(self):
        print("Vehicle arming...")
        self.vehicle.armed = True
        while not self.vehicle.armed:
            time.sleep(.1)        
        print("Vehicle armed")

    def takeoff(self,altitude):
        print("Vehicle taking off: "+str(altitude))
        self.vehicle.simple_takeoff(altitude)
        print("Vehicle took off")

    def state(self):
        self.vehicle.wait_ready('autopilot_version')

        # Get all vehicle attributes (state)
        print("\nGet all vehicle attribute values:")
        print(" Autopilot Firmware version: %s" % self.vehicle.version)
        print("   Major version number: %s" % self.vehicle.version.major)
        print("   Minor version number: %s" % self.vehicle.version.minor)
        print("   Patch version number: %s" % self.vehicle.version.patch)
        print("   Release type: %s" % self.vehicle.version.release_type())
        print("   Release version: %s" % self.vehicle.version.release_version())
        print("   Stable release?: %s" % self.vehicle.version.is_stable())
        print(" Autopilot capabilities")
        print("   Supports MISSION_FLOAT message type: %s" % self.vehicle.capabilities.mission_float)
        print("   Supports PARAM_FLOAT message type: %s" % self.vehicle.capabilities.param_float)
        print("   Supports MISSION_INT message type: %s" % self.vehicle.capabilities.mission_int)
        print("   Supports COMMAND_INT message type: %s" % self.vehicle.capabilities.command_int)
        print("   Supports PARAM_UNION message type: %s" % self.vehicle.capabilities.param_union)
        print("   Supports ftp for file transfers: %s" % self.vehicle.capabilities.ftp)
        print("   Supports commanding attitude offboard: %s" % self.vehicle.capabilities.set_attitude_target)
        print("   Supports commanding position and velocity targets in local NED frame: %s" % self.vehicle.capabilities.set_attitude_target_local_ned)
        print("   Supports set position + velocity targets in global scaled integers: %s" % self.vehicle.capabilities.set_altitude_target_global_int)
        print("   Supports terrain protocol / data handling: %s" % self.vehicle.capabilities.terrain)
        print("   Supports direct actuator control: %s" % self.vehicle.capabilities.set_actuator_target)
        print("   Supports the flight termination command: %s" % self.vehicle.capabilities.flight_termination)
        print("   Supports mission_float message type: %s" % self.vehicle.capabilities.mission_float)
        print("   Supports onboard compass calibration: %s" % self.vehicle.capabilities.compass_calibration)
        print(" Global Location: %s" % self.vehicle.location.global_frame)
        print(" Global Location (relative altitude): %s" % self.vehicle.location.global_relative_frame)
        print(" Local Location: %s" % self.vehicle.location.local_frame)
        print(" Attitude: %s" % self.vehicle.attitude)
        print(" Velocity: %s" % self.vehicle.velocity)
        print(" GPS: %s" % self.vehicle.gps_0)
        print(" Gimbal status: %s" % self.vehicle.gimbal)
        print(" Battery: %s" % self.vehicle.battery)
        print(" EKF OK?: %s" % self.vehicle.ekf_ok)
        print(" Last Heartbeat: %s" % self.vehicle.last_heartbeat)
        print(" Rangefinder: %s" % self.vehicle.rangefinder)
        print(" Rangefinder distance: %s" % self.vehicle.rangefinder.distance)
        print(" Rangefinder voltage: %s" % self.vehicle.rangefinder.voltage)
        print(" Heading: %s" % self.vehicle.heading)
        print(" Is Armable?: %s" % self.vehicle.is_armable)
        print(" System status: %s" % self.vehicle.system_status.state)
        print(" Groundspeed: %s" % self.vehicle.groundspeed)    # settable
        print(" Airspeed: %s" % self.vehicle.airspeed)    # settable
        print(" Mode: %s" % self.vehicle.mode.name)    # settable
        print(" Armed: %s" % self.vehicle.armed)    # settable

        
    #Flight control by image processing here
    def set_rc_channel_pwm(self,channel_id, pwm=1500):
        if channel_id < 1 or channel_id > 18:
            print("channel does not exist.")
            return
        rc_channel_values = [65535 for _ in range(18)]
        rc_channel_values[channel_id - 1] = pwm
        print("channel: "+str(channel_id)+" pwm value: "+str(pwm))

        self.vehicle.channels.overrides[str(channel_id)] = pwm          
        # Get all original channel values (after override)
        print("\nChannel overrides: %s" % self.vehicle.channels.overrides)

    #Rudder control
    def control_x(self,x_error):
        self.current_x_deg = self.current_x_deg + x_error/self.pix_per_x_deg
        if(self.current_x_deg < self.min_x_deg):
            self.current_x_deg = self.min_x_deg
        if(self.current_x_deg > self.max_x_deg):
            self.current_x_deg = self.max_x_deg    
            
        x_pwm = int(self.pwm_at_90_x*self.current_x_deg/90)
        self.set_rc_channel_pwm(self.servo_ch_rudder,x_pwm)
        print("x degree: "+str(self.current_x_deg)+" PWM:" +str(x_pwm))

    #Elevator control
    def control_y(self,y_error):
        self.current_y_deg = self.current_y_deg + y_error/self.pix_per_y_deg
        if(self.current_y_deg < self.min_y_deg):
            self.current_y_deg = self.min_y_deg
        if(self.current_y_deg > self.max_y_deg):
            self.current_y_deg = self.max_y_deg         
             
        y_pwm = int(self.pwm_at_90_y*self.current_y_deg/90)    
        self.set_rc_channel_pwm(self.servo_ch_elevator,y_pwm)
        print("y degree: "+str(self.current_y_deg)+" PWM:" +str(y_pwm))        

##################### Drone Control-via Mavlink ##########################
class DroneMavControl(object):
    # Connect to the Vehicle
    conn_string = '/dev/ttyAMA0'
    servo_ch_elevator = 2
    servo_ch_rudder = 4
    pwm_at_90_x = 1600 # servo position when flying straight
    pwm_at_90_y = 1600 
    
    default_x_deg = 90 #goes from 0 to 180
    default_y_deg = 90 #goes from 0 to 180
    current_x_deg = default_x_deg 
    current_y_deg = default_y_deg
    pix_per_x_deg = 180 # used to adjust movement sensitivity
    pix_per_y_deg = 50 # used to adjust movement sensitivity
    max_x_deg = default_x_deg + 45
    min_x_deg = default_x_deg - 45
    max_y_deg = default_y_deg + 45
    min_y_deg = default_y_deg - 45    
        
    def __init__(self):
        self.the_conn = mavutil.mavlink_connection(self.conn_string,baud=57600)
        print("Connecting to vehicle on: " + str(self.conn_string))  
        self.the_conn.wait_heartbeat()
        print("Heartbeat from system ( system %u component %u)" % (self.the_conn.target_system, self.the_conn.target_component))
        self.disarm()
        time.sleep(3)  

    # Preflight/launch
    def disarm(self):
        #DISARM = 0, 0, 0, 0, 0, 0, 0, 0
        self.the_conn.mav.command_long_send(self.the_conn.target_system, self.the_conn.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
        msg = self.the_conn.recv_match(type='COMMAND_ACK', blocking=True)
        print(msg)

    def arm(self):
        #ARM = 0, 1, 0, 0, 0, 0, 0, 0
        self.the_conn.mav.command_long_send(self.the_conn.target_system, self.the_conn.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        msg = self.the_conn.recv_match(type='COMMAND_ACK', blocking=True)
        print(msg)

    def takeoff(self,altitude):
        #Take off to given altitude meters
        self.the_conn.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(altitude,self.the_conn.target_system, self.the_conn.target_component,mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b110111111000) , 20,0,-10,0,0,0,0,0,0,0,0))
        #msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
        #print(msg)

    def state(self):
        print("here comes the states: ")
        mode = 'ACRO'
        if mode not in self.the_conn.mode_mapping():
            print('Unknown mode : {}'.format(mode))
            print('Try:', list(self.the_conn.mode_mapping().keys()))
            exit(1)
        
        mode_id = self.the_conn.mode_mapping()[mode]
        self.the_conn.mav.set_mode_send(self.the_conn.target_system,mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,mode_id)

    #Flight control by image processing here
    def set_rc_channel_pwm(self,channel_id, pwm=1500):
        if channel_id < 1 or channel_id > 18:
            print("channel does not exist.")
            return
        rc_channel_values = [65535 for _ in range(8)]
        rc_channel_values[channel_id - 1] = pwm
        print(rc_channel_values)
        self.the_conn.set_servo(channel_id,pwm)
        self.the_conn.mav.rc_channels_override_send(self.the_conn.target_system, self.the_conn.target_component, *rc_channel_values)

    #Rudder control
    def control_x(self,x_error):
        self.current_x_deg = self.current_x_deg + x_error/self.pix_per_x_deg
        if(self.current_x_deg < self.min_x_deg):
            self.current_x_deg = self.min_x_deg
        if(self.current_x_deg > self.max_x_deg):
            self.current_x_deg = self.max_x_deg    
            
        x_pwm = int(self.pwm_at_90_x*self.current_x_deg/90)
        self.set_rc_channel_pwm(self.servo_ch_rudder,x_pwm)
        print("x degree: "+str(self.current_x_deg)+" PWM:" +str(x_pwm))

    #Elevator control
    def control_y(self,y_error):
        self.current_y_deg = self.current_y_deg + y_error/self.pix_per_y_deg
        if(self.current_y_deg < self.min_y_deg):
            self.current_y_deg = self.min_y_deg
        if(self.current_y_deg > self.max_y_deg):
            self.current_y_deg = self.max_y_deg         
             
        y_pwm = int(self.pwm_at_90_y*self.current_y_deg/90)    
        self.set_rc_channel_pwm(self.servo_ch_elevator,y_pwm)
        print("y degree: "+str(self.current_y_deg)+" PWM:" +str(y_pwm))

##################### Webcam Interface ##########################

# Define VideoStream class to handle streaming of video from webcam in separate processing thread
# Source - Adrian Rosebrock, PyImageSearch: https://www.pyimagesearch.com/2015/12/28/increasing-raspberry-pi-fps-with-python-and-opencv/
class VideoStream:
    """Camera object that controls video streaming from the Picamera"""
    def __init__(self,resolution=(640,480),framerate=30):
        # Initialize the PiCamera and the camera image stream
        self.stream = cv2.VideoCapture(0)
        ret = self.stream.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        ret = self.stream.set(3,resolution[0])
        ret = self.stream.set(4,resolution[1])
            
        # Read first frame from the stream
        (self.grabbed, self.frame) = self.stream.read()

	# Variable to control when the camera is stopped
        self.stopped = False

    def start(self):
	# Start the thread that reads frames from the video stream
        Thread(target=self.update,args=()).start()
        return self

    def update(self):
        # Keep looping indefinitely until the thread is stopped
        while True:
            # If the camera is stopped, stop the thread
            if self.stopped:
                # Close camera resources
                self.stream.release()
                return

            # Otherwise, grab the next frame from the stream
            (self.grabbed, self.frame) = self.stream.read()

    def read(self):
	# Return the most recent frame
        return self.frame

    def stop(self):
	# Indicate that the camera and thread should be stopped
        self.stopped = True


##################### Main ##########################

class VideoDetectTrack:
    # List of objects to be detected
    objects=['airplane','person']
    #subdir name.  subdirectory to put the following two files
    MODEL_NAME = 'Sample_TFLite_model'
    #graph file name
    GRAPH_NAME = 'edgetpu.tflite'
    #label file name  
    LABELMAP_NAME = 'labelmap.txt'
    min_conf_threshold = 0.5
    imW = 1280
    imH = 720

    x_max = imW #1280 #640  # width of pi camera image
    y_max = imH #720 #480   # height of pi camera image
    x_center = int(x_max / 2)  # center value
    y_center = int(y_max / 2)  # center value
    height = 100
    width = 100
    floating_model = True
    input_mean = 127.5
    input_std = 127.5    
    
    # Get path to current working directory
    CWD_PATH = os.getcwd()

    # Path to .tflite file, which contains the model that is used for object detection
    PATH_TO_CKPT = os.path.join(CWD_PATH,MODEL_NAME,GRAPH_NAME)
    # Path to label map file
    PATH_TO_LABELS = os.path.join(CWD_PATH,MODEL_NAME,LABELMAP_NAME)
    interpreter = Interpreter(model_path=PATH_TO_CKPT,experimental_delegates=[load_delegate('libedgetpu.so.1.0')])
    interpreter.allocate_tensors()
    # Get model details
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
    boxes_idx, classes_idx, scores_idx = 0, 1, 2    
                
    # Load the label map
    with open(PATH_TO_LABELS, 'r') as f:
        labels = [line.strip() for line in f.readlines()]

    def __init__(self):
        # Have to do a weird fix for label map if using the COCO "starter model" from
        # https://www.tensorflow.org/lite/models/object_detection/overview
        # First label is '???', which has to be removed.
        if self.labels[0] == '???':
            del(self.labels[0])

        print("Model file location: "+self.PATH_TO_CKPT)
        print("Label file location: "+self.PATH_TO_LABELS)
                
        self.height = self.input_details[0]['shape'][1]
        self.width = self.input_details[0]['shape'][2]
        print("height: ")
        print(self.height)
        print("width: ")
        print(self.width)

        self.floating_model = (self.input_details[0]['dtype'] == np.float32)
        print(self.floating_model)

        # Check output layer name to determine if this model was created with TF2 or TF1,
        # because outputs are ordered differently for TF2 and TF1 models
        outname = self.output_details[0]['name']

        if ('StatefulPartitionedCall' in outname): # This is a TF2 model
            self.boxes_idx, self.classes_idx, self.scores_idx = 1, 3, 0
        else: # This is a TF1 model
            self.boxes_idx, self.classes_idx, self.scores_idx = 0, 1, 2

    # print object coordinates
    def mapObjectPosition(self,x, y):
        print("[INFO] Object Center coordinates at X0 = {0} and Y0 =  {1}".format(x, y))
        print("Center of Image: "+ str(self.x_center)+ "," +str(self.y_center))

    def tracking(self,x,y): #x and y are the position from image processing
        print("object tracking")

        x_error = x - self.x_center  # calculating the x error in the distance of the ball from the center of the image
        y_error = y - self.y_center  # calculating the x error in the distance of the ball from the center of the image

        print("[INFO] Object Center coordinates at X0 = {0} and Y0 =  {1}".format(x, y))
        print("X error: " +str(x_error))
        print("Y error: " +str(y_error))

        
    def detection(self,frame1):  # Grab frame from video stream
        wd = self.width
        ht = self.height
        # Acquire frame and resize to expected shape [1xHxWx3]
        frame = frame1.copy()
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame_resized = cv2.resize(frame_rgb, (wd, ht))
        input_data = np.expand_dims(frame_resized, axis=0)

        # Normalize pixel values if using a floating model (i.e. if model is non-quantized)
        if self.floating_model:
            input_data = (np.float32(input_data) - self.input_mean) / self.input_std

        #print(self.input_details[0]['index'])
        #print(input_data)
        # Perform the actual detection by running the model with the image as input
        self.interpreter.set_tensor(self.input_details[0]['index'],input_data)
        self.interpreter.invoke()

        # Retrieve detection results
        classes = self.interpreter.get_tensor(self.output_details[self.classes_idx]['index'])[0] # Class index of detected objects
        boxes = self.interpreter.get_tensor(self.output_details[self.boxes_idx]['index'])[0] # Bounding box coordinates of detected objects
        scores = self.interpreter.get_tensor(self.output_details[self.scores_idx]['index'])[0] # Confidence of detected objects

        # Draw a circle in the middle of the frame
        cv2.circle(frame, (self.x_center,self.y_center), 20,(0, 255, 255), 2)

        # Loop over all detections and draw detection box 
        scores_max = 0
        i_max = 0        
        for i in range(len(scores)):
            #if confidence is above minimum threshold
            if ((scores[i] > self.min_conf_threshold) and (scores[i] <= 1.0)):
                #if we found what we are looking for
                if self.labels[int(classes[i])] in self.objects:
                    if (scores_max < scores[i]):
                        scores_max = scores[i]
                        i_max = i
    
        i = i_max  
        if ((scores[i] > self.min_conf_threshold) and (scores[i] <= 1.0) and self.labels[int(classes[i])] in self.objects):              
            print("found: "+ self.labels[int(classes[i])])
            # Get bounding box coordinates and draw box
            # Interpreter can return coordinates that are outside of image dimensions, need to force them to be within image using max() and min()
            ymin = int(max(1,(boxes[i][0] * self.imH)))
            xmin = int(max(1,(boxes[i][1] * self.imW)))
            ymax = int(min(self.imH,(boxes[i][2] * self.imH)))
            xmax = int(min(self.imW,(boxes[i][3] * self.imW)))
            cv2.rectangle(frame, (xmin,ymin), (xmax,ymax), (10, 255, 0), 2)

            # Draw label
            object_name = self.labels[int(classes[i])] # Look up object name from "labels" array using class index
            label = '%s: %d%%' % (object_name, int(scores[i]*100)) # Example: 'person: 72%'
            labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2) # Get font size
            label_ymin = max(ymin, labelSize[1] + 10) # Make sure not to draw label too close to top of window
            cv2.rectangle(frame, (xmin, label_ymin-labelSize[1]-10), (xmin+labelSize[0], label_ymin+baseLine-10), (255, 255, 255), cv2.FILLED) # Draw white box to put label text in
            cv2.putText(frame, label, (xmin, label_ymin-7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2) # Draw label text

            x = xmin +int((xmax-xmin)/2)
            y = ymin +int((ymax-ymin)/2)
            cv2.circle(frame, (x,y), 20,(0, 255, 255), 2)
            self.mapObjectPosition(x,y)
            self.tracking(x,y)


            print("x: "+str(x))
            print("x_center: "+str(self.x_center))
            print("y: "+str(y))
            print("y_center: "+str(self.y_center))   
        
            #comment out the gimbal here              
            cameragimbal.moveX(x-self.x_center)
            cameragimbal.moveY(self.y_center-y)        #image is flipped vertically
        
            # Command vehicle to change          
            drone.control_x(x-self.x_center)   
            drone.control_y(self.y_center-y)    

        # Draw framerate in corner of frame
        cv2.putText(frame,'FPS: {0:.2f}'.format(frame_rate_calc),(30,50),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,0),2,cv2.LINE_AA)

        # All the results have been drawn on the frame, so it's time to display it.
        cv2.imshow('Object detector', frame)


############ Main Funtion ############

# Initialize frame rate calculation
frame_rate_calc = 1
freq = cv2.getTickFrequency()

# Initialize video stream
videostream = VideoStream(resolution=(1280,720),framerate=30).start()
time.sleep(1)

#Gimbal servo initialize
cameragimbal = ServoKit()

object_detect = VideoDetectTrack()

#DroneControl initialize
#via MAVLink
#drone = DroneMavControl()
#via DroneKit
drone = DroneKitControl()
drone.arm()
drone.takeoff(30)
drone.state()

while True == True:
    # Start timer (for calculating frame rate)
    t1 = cv2.getTickCount()
    
    object_detect.detection(videostream.read())
    
    # Calculate framerate
    t2 = cv2.getTickCount()
    time1 = (t2-t1)/freq
    frame_rate_calc= 1/time1
    # Press 'q' to quit
    if cv2.waitKey(1) == ord('q'):
        break

# Clean up
cv2.destroyAllWindows()
videostream.stop()


print("Completed")
