######## Webcam Object Detection Using Tensorflow-trained Classifier #########
######## Command arduplane over MAVLink ########
# To run:
# python3 TFLite_detection_webcam.py --modeldir=Sample_TFLite_model --edgetpu
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


##################### Drone Control ##########################
class DroneControl(object):
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

    #Flight control by image processing here
    def set_rc_channel_pwm(self,channel_id, pwm=1500):
        if channel_id < 1 or channel_id > 18:
            print("channel does not exist.")
            return
        rc_channel_values = [65535 for _ in range(18)]
        rc_channel_values[channel_id - 1] = pwm
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


##################### Main Parsing ##########################

# Define and parse input arguments
parser = argparse.ArgumentParser()
parser.add_argument('--modeldir', help='Folder the .tflite file is located in',
                    required=True)
parser.add_argument('--graph', help='Name of the .tflite file, if different than detect.tflite',
                    default='detect.tflite')
parser.add_argument('--labels', help='Name of the labelmap file, if different than labelmap.txt',
                    default='labelmap.txt')
parser.add_argument('--threshold', help='Minimum confidence threshold for displaying detected objects',
                    default=0.5)
parser.add_argument('--resolution', help='Desired webcam resolution in WxH. If the webcam does not support the resolution entered, errors may occur.',
                    default='1280x720')
parser.add_argument('--edgetpu', help='Use Coral Edge TPU Accelerator to speed up detection',
                    action='store_true')

args = parser.parse_args()

MODEL_NAME = args.modeldir
GRAPH_NAME = args.graph
LABELMAP_NAME = args.labels
min_conf_threshold = float(args.threshold)
resW, resH = args.resolution.split('x')
imW, imH = int(resW), int(resH)
use_TPU = args.edgetpu

##################### TensorFlow ##########################
# Import TensorFlow libraries
# If tflite_runtime is installed, import interpreter from tflite_runtime, else import from regular tensorflow
# If using Coral Edge TPU, import the load_delegate library
pkg = importlib.util.find_spec('tflite_runtime')
if pkg:
    from tflite_runtime.interpreter import Interpreter
    if use_TPU:
        from tflite_runtime.interpreter import load_delegate
else:
    from tensorflow.lite.python.interpreter import Interpreter
    if use_TPU:
        from tensorflow.lite.python.interpreter import load_delegate

# If using Edge TPU, assign filename for Edge TPU model
if use_TPU:
    # If user has specified the name of the .tflite file, use that name, otherwise use default 'edgetpu.tflite'
    if (GRAPH_NAME == 'detect.tflite'):
        GRAPH_NAME = 'edgetpu.tflite'       

# Get path to current working directory
CWD_PATH = os.getcwd()

# Path to .tflite file, which contains the model that is used for object detection
PATH_TO_CKPT = os.path.join(CWD_PATH,MODEL_NAME,GRAPH_NAME)

# Path to label map file
PATH_TO_LABELS = os.path.join(CWD_PATH,MODEL_NAME,LABELMAP_NAME)

# Load the label map
with open(PATH_TO_LABELS, 'r') as f:
    labels = [line.strip() for line in f.readlines()]

# Have to do a weird fix for label map if using the COCO "starter model" from
# https://www.tensorflow.org/lite/models/object_detection/overview
# First label is '???', which has to be removed.
if labels[0] == '???':
    del(labels[0])

# Load the Tensorflow Lite model.
# If using Edge TPU, use special load_delegate argument
if use_TPU:
    interpreter = Interpreter(model_path=PATH_TO_CKPT,experimental_delegates=[load_delegate('libedgetpu.so.1.0')])
    print(PATH_TO_CKPT)
else:
    interpreter = Interpreter(model_path=PATH_TO_CKPT)

interpreter.allocate_tensors()

# Get model details
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
height = input_details[0]['shape'][1]
width = input_details[0]['shape'][2]

floating_model = (input_details[0]['dtype'] == np.float32)

input_mean = 127.5
input_std = 127.5

# Check output layer name to determine if this model was created with TF2 or TF1,
# because outputs are ordered differently for TF2 and TF1 models
outname = output_details[0]['name']

if ('StatefulPartitionedCall' in outname): # This is a TF2 model
    boxes_idx, classes_idx, scores_idx = 1, 3, 0
else: # This is a TF1 model
    boxes_idx, classes_idx, scores_idx = 0, 1, 2

##################### Object Position/Tracking ##########################

# print object coordinates
def mapObjectPosition(x, y):
    print("[INFO] Object Center coordinates at X0 = {0} and Y0 =  {1}".format(x, y))

x_max = 1280 #640  # width of pi camera image
y_max = 720 #480   # height of pi camera image
x_center = int(x_max / 2)  # center value
y_center = int(y_max / 2)  # center value
print("Center of Image: "+ str(x_center)+ "," +str(y_center))



def object_tracking(x,y): #x and y are the position from image processing
    print("object tracking")

    x_error = x - x_center  # calculating the x error in the distance of the ball from the center of the image
    y_error = y - y_center  # calculating the x error in the distance of the ball from the center of the image

    print("[INFO] Object Center coordinates at X0 = {0} and Y0 =  {1}".format(x, y))
    print("X error: " +str(x_error))
    print("Y error: " +str(y_error))

##################### Object Tracking ##########################
# List of objects to be detected
objects=['airplane','person']
# Definition for OpenCV object detection
def object_detection():  # needs to be modified so definition can be called as part of main function
    # Grab frame from video stream
    frame1 = videostream.read()

    # Acquire frame and resize to expected shape [1xHxWx3]
    frame = frame1.copy()
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    frame_resized = cv2.resize(frame_rgb, (width, height))
    input_data = np.expand_dims(frame_resized, axis=0)

    # Normalize pixel values if using a floating model (i.e. if model is non-quantized)
    if floating_model:
        input_data = (np.float32(input_data) - input_mean) / input_std

    # Perform the actual detection by running the model with the image as input
    interpreter.set_tensor(input_details[0]['index'],input_data)
    interpreter.invoke()

    # Retrieve detection results
    classes = interpreter.get_tensor(output_details[classes_idx]['index'])[0] # Class index of detected objects
    boxes = interpreter.get_tensor(output_details[boxes_idx]['index'])[0] # Bounding box coordinates of detected objects
    scores = interpreter.get_tensor(output_details[scores_idx]['index'])[0] # Confidence of detected objects

    # Draw a circle in the middle of the frame
    cv2.circle(frame, (x_center,y_center), 20,(0, 255, 255), 2)

    # Loop over all detections and draw detection box 
    scores_max = 0
    i_max = 0        
    for i in range(len(scores)):
        #if confidence is above minimum threshold
        if ((scores[i] > min_conf_threshold) and (scores[i] <= 1.0)):
            #if we found what we are looking for
            if labels[int(classes[i])] in objects:
                if (scores_max < scores[i]):
                    scores_max = scores[i]
                    i_max = i
    
    i = i_max  
    if ((scores[i] > min_conf_threshold) and (scores[i] <= 1.0) and labels[int(classes[i])] in objects):              
        print("found: "+ labels[int(classes[i])])
        # Get bounding box coordinates and draw box
        # Interpreter can return coordinates that are outside of image dimensions, need to force them to be within image using max() and min()
        ymin = int(max(1,(boxes[i][0] * imH)))
        xmin = int(max(1,(boxes[i][1] * imW)))
        ymax = int(min(imH,(boxes[i][2] * imH)))
        xmax = int(min(imW,(boxes[i][3] * imW)))
        cv2.rectangle(frame, (xmin,ymin), (xmax,ymax), (10, 255, 0), 2)

        # Draw label
        object_name = labels[int(classes[i])] # Look up object name from "labels" array using class index
        label = '%s: %d%%' % (object_name, int(scores[i]*100)) # Example: 'person: 72%'
        labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2) # Get font size
        label_ymin = max(ymin, labelSize[1] + 10) # Make sure not to draw label too close to top of window
        cv2.rectangle(frame, (xmin, label_ymin-labelSize[1]-10), (xmin+labelSize[0], label_ymin+baseLine-10), (255, 255, 255), cv2.FILLED) # Draw white box to put label text in
        cv2.putText(frame, label, (xmin, label_ymin-7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2) # Draw label text

        x = xmin +int((xmax-xmin)/2)
        y = ymin +int((ymax-ymin)/2)
        cv2.circle(frame, (x,y), 20,(0, 255, 255), 2)
        mapObjectPosition(x,y)
        object_tracking(x,y)


        print("x: "+str(x))
        print("x_center: "+str(x_center))
        print("y: "+str(y))
        print("y_center: "+str(y_center))   
        
        #comment out the gimbal here              
        cameragimbal.moveX(x-x_center)
        cameragimbal.moveY(y_center-y)        #image is flipped vertically
        
        # Command vehicle to change          
        drone.control_x(x-x_center)   
        drone.control_y(y_center-y)    

    # Draw framerate in corner of frame
    cv2.putText(frame,'FPS: {0:.2f}'.format(frame_rate_calc),(30,50),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,0),2,cv2.LINE_AA)

    # All the results have been drawn on the frame, so it's time to display it.
    cv2.imshow('Object detector', frame)


# Initialize frame rate calculation
frame_rate_calc = 1
freq = cv2.getTickFrequency()

# Initialize video stream
videostream = VideoStream(resolution=(imW,imH),framerate=30).start()
time.sleep(1)

#Gimbal servo initialize
cameragimbal = ServoKit()
 

#DroneControl initialize
drone = DroneControl()

while True == True:
    # Start timer (for calculating frame rate)
    t1 = cv2.getTickCount()
    
    object_detection()
    
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

# Close vehicle object
print("Close vehicle object")
vehicle.close()

print("Completed")
