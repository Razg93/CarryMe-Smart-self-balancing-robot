from pose_utils import mod, sigmoid, sigmoid_and_argmax2d, get_offset_point, get_offsets, draw_lines
import os
import argparse
import cv2
import numpy as np
import sys
import pdb
import time
import math
import pathlib
from threading import Thread
import importlib.util
import datetime
import pyrealsense2 as rs
import matplotlib.pyplot as plt
import serial # Module needed for serial communication
import time
import json

def initilaize_camera():

    config = rs.config()
    config.enable_stream(rs.stream.depth, 320, 240, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    pipe = rs.pipeline()
    profile = pipe.start(config)
    # Skip 5 first frames to give the Auto-Exposure time to adjust
    for x in range(5):
        pipe.wait_for_frames()
    
    frameset = pipe.wait_for_frames()

    color_frame = frameset.get_color_frame()
    depth_frame = frameset.get_depth_frame()
    color_init = np.asanyarray(color_frame.get_data())

    return pipe, profile,frameset, color_frame, depth_frame, color_init

pipe, profile,frameset, color_frame, depth_frame, color_init = initilaize_camera()


# Import TensorFlow libraries
# If tensorflow is not installed, import interpreter from tflite_runtime, else import from regular tensorflow
# If using Coral Edge TPU, import the load_delegate library
pkg = importlib.util.find_spec('tensorflow')
use_TPU = False
if pkg is None:
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
        
# Path to .tflite file, which contains the model that is used for object detection
PATH_TO_CKPT = "/home/pi/Desktop/posepon/posenet_mobilenet_v1_100_257x257_multi_kpt_stripped.tflite"

# If using Edge TPU, use special load_delegate argument
if use_TPU:
    interpreter = Interpreter(model_path=PATH_TO_CKPT,
                              experimental_delegates=[load_delegate('libedgetpu.so.1.0')])
    print(PATH_TO_CKPT)
else:
    interpreter = Interpreter(model_path=PATH_TO_CKPT)
interpreter.allocate_tensors()

# Get model details
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
#print(input_details)
#print(output_details)
height = input_details[0]['shape'][1]
width = input_details[0]['shape'][2]
#set stride to 32 based on model size
output_stride = 32

#led_on = False
floating_model = (input_details[0]['dtype'] == np.float32)
min_conf_threshold = 0.5

input_mean = 127.5
input_std = 127.5


dist_to_person = 0.5
dist_to_stop = 0.5
object_to_track = [5,6]




def circle(frame, center_coordinates):
    radius = 2
    color = (0, 255, 0)
    thickness = 2
    return cv2.circle(frame, center_coordinates, radius, color, thickness)


def send_value(commands_json):
    json_object = json.dumps(commands_json,indent = 5)
    try:
        with open('commands.txt','w') as outfile:
            outfile.write(json_object)
    except:
        print("couldnt write to json file")

def get_distance_to_obj(depth_frame, x, y, dist):
    try:
        depth_to_object = depth_frame.get_distance(x,y)
    except:
        print("cannot validate the rang to the object")
        depth_to_object = dist 
    return depth_to_object

def get_center_coordinates(startX, startY, endX, endY):
    x_center = int((startX + endX) / 2)
    y_center = int((startY + endY) / 2)
    return x_center, y_center

def main():
    fps=1

    while True:
        start_time=time.time()
        
        #----------------Capture Camera Frame-----------------
        frameset = pipe.wait_for_frames()
        color_frame = frameset.get_color_frame()
        depth_frame = frameset.get_depth_frame()

        
        color_image = np.asanyarray(color_frame.get_data())
        
        colorizer = rs.colorizer()
        colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())
      
        # Create alignment primitive with color as its target stream:
        align = rs.align(rs.stream.color)
        frameset = align.process(frameset)
        
        # Update color and depth frames:
        aligned_depth_frame = frameset.get_depth_frame()

        colorized_depth = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())
        depth = np.asanyarray(aligned_depth_frame.get_data())
    

        # Acquire frame and resize to expected shape [1xHxWx3]
        frame = color_image.copy()
        

        #-------------------Inference---------------------------------
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame_resized = cv2.resize(frame_rgb, (width, height))
        input_data = np.expand_dims(frame_resized, axis=0)
        
        frame_resized = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2RGB)

        # Normalize pixel values if using a floating model (i.e. if model is non-quantized)
        if floating_model:
            input_data = (np.float32(input_data) - input_mean) / input_std

        # Perform the actual detection by running the model with the image as input
        interpreter.set_tensor(input_details[0]['index'],input_data)
        interpreter.invoke()
        #num = interpreter.get_tensor(output_details[3]['index'])[0]
        #get y,x positions from heatmap
        coords = sigmoid_and_argmax2d(output_details, min_conf_threshold, output_details, interpreter)
        #keep track of keypoints that don't meet threshold
        drop_pts = list(np.unique(np.where(coords ==0)[0]))
        #get offets from postions
        offset_vectors = get_offsets(output_details, coords, interpreter)
        
        #use stide to get coordinates in image coordinates
        keypoint_positions = coords * output_stride + offset_vectors
        depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        depth_res = depth * depth_scale
        depth_res = depth_res[depth_res!=0]
        
       
        if depth_res.size == 0:
          continue
        
        dist = depth_res.min()
        print("Dist: " + str("{0:.2f}").format(dist))
        
        commands_json = {"RIGHT":0, "LEFT":0,"BACKWARD":0,"FORWARD":0,"NOTHING":1}
        send_value(commands_json)
  
        # Loop over all detections and draw detection box if confidence is above minimum threshold
        for i in range(len(keypoint_positions)):
            
            #don't draw low confidence points
            if i  in drop_pts:
                continue
            # Center coordinates
            x_center = int(keypoint_positions[i][1])

            y_center = int(keypoint_positions[i][0])
            center_coordinates = (x_center, y_center)
            radius = 2
            color = (0, 255, 0)
            thickness = 2
            #cv2.circle(frame_resized, center_coordinates, radius, color, thickness)
            debug = True
            if (i == 11 or i == 12) and dist > dist_to_stop:

                """object deteceted by the model
                    and the way is open. start follow"""
                print("hello person")
                cv2.circle(frame_resized, center_coordinates, radius, color, thickness)
                #depth_to_object = depth_frame.get_distance(x_center,y_center)
                depth_to_object = get_distance_to_obj(depth_frame, x_center, y_center, dist)
                text = "depth_to_object: " + str("{0:.2f}").format(depth_to_object)
                print(text)
                print(x_center)
                print(y_center)
                if x_center > 150:#object right from the center
                    xAxis = 170
                    yAxis = 140
                    print("right")
                    commands_json = {"RIGHT":1, "LEFT":0,"BACKWARD":0,"FORWARD":0,"NOTHING":0}
                    send_value(commands_json)           
                                         
                elif x_center < 90:#object left from the center
                    xAxis = 110
                    yAxis = 140
                    print("left")
                    commands_json = {"RIGHT":0, "LEFT":1,"BACKWARD":0,"FORWARD":0,"NOTHING":0}
                    send_value(commands_json)

                elif 90 < x_center < 150 and depth_to_object > dist_to_person:#object is centered and far from object
                    Axis = 140
                    Axis = 110
                    print("forward")
                    commands_json = {"RIGHT":0, "LEFT":0,"BACKWARD":0,"FORWARD":1,"NOTHING":0}
                    send_value(commands_json)

                elif 90 < x_center < 150 and 0 < depth_to_object < dist_to_person:#object is centered but close object
                    xAxis = 140
                    yAxis = 140
                    print("stop")
                    commands_json = {"RIGHT":0, "LEFT":0,"BACKWARD":0,"FORWARD":0,"NOTHING":1}
                    send_value(commands_json)
                    
#             if (i == 11 or i == 12) and dist < dist_to_stop:
#                 """object deteceted by the model
#                 but the way is blocked. stop stay stable!"""
#                 print("object detected, but can't move something is blocking the way.")
#                 circle(frame_resized, center_coordinates)
#                 xAxis = 140
#                 yAxis = 140
#                 commands_json = {"RIGHT":0, "LEFT":0,"BACKWARD":0,"FORWARD":0,"NOTHING":1}
#                 send_value(commands_json)
#                 depth_to_object = get_distance_to_obj(depth_frame, x_center, y_center, dist)
#                 print("depth_to_object: " + str("{0:.2f}").format(depth_to_object))
                
                 
#             if i != object_to_track:
#                 """object not deteceted by the model
#                  stop stay stable!"""
#                 print("object not detected")
#                 circle(frame_resized, center_coordinates)
#                 commands_json = {"RIGHT":0, "LEFT":0,"BACKWARD":0,"FORWARD":0,"NOTHING":1}
#                 send_value(commands_json)
#                 depth_to_object = get_distance_to_obj(depth_frame, x_center, y_center, dist)
#                 print("depth_to_object: " + str("{0:.2f}").format(depth_to_object))
                
                
            
#             else:
#                 print("dont move")
#                 xAxis = 140
#                 yAxis = 140
#                 commands_json = {"RIGHT":0, "LEFT":0,"BACKWARD":0,"FORWARD":0,"NOTHING":1}
#                 send_value(commands_json)
                    
            if debug:
                cv2.putText(frame_resized, str(i), (x_center-4, y_center-4), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 1) # Draw label text
                
        frame_resized = draw_lines(keypoint_positions, frame_resized, drop_pts)
        print(frame_resized.shape)
        
        fps = round(1.0 / (time.time() - start_time),1)
        # Draw framerate in corner of frame
        cv2.putText(frame,'FPS: {0:.2f}'.format(fps),(30,50),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),2,cv2.LINE_AA)
        # All the results have been drawn on the frame, so it's time to display it.
        cv2.imshow('Object detector', frame)
        cv2.imshow('colorized_depth1', colorized_depth)
        cv2.imshow('frame_resized', frame_resized)

        # Press 'q' to quit
        if cv2.waitKey(1) == ord('q'):
            break

# Clean up
cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
