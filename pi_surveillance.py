# import the necessary packages
from pyimagesearch.tempimage import TempImage
from picamera.array import PiRGBArray
from picamera import PiCamera
from shutil import copy
import paho.mqtt.client as mqtt
import argparse
import warnings
import datetime
import imutils
import json
import time
import cv2
import os

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-c", "--conf", required=True,
    help="path to the JSON configuration file")
args = vars(ap.parse_args())
 
# filter warnings, load the configuration and initialize the Dropbox
# client
warnings.filterwarnings("ignore")
conf = json.load(open(args["conf"]))
client = None

#connect to mqtt
mqtt_client = mqtt.Client(conf["mqtt_username"]) 
mqtt_client.username_pw_set(conf["mqtt_username"], conf["mqtt_password"])
mqtt_client.connect(conf["mqtt_broker"])
mqtt_timeout = datetime.datetime.now()
mqtt_client.publish("homeassistant/binary_sensor/picam/config", '{"name": "picam", "unique_id": "picam", "device_class": "motion", "state_topic": "homeassistant/binary_sensor/picam/state"}', retain=True)
mqtt_last_state = ""
 
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = tuple(conf["resolution"])
camera.framerate = conf["fps"]
rawCapture = PiRGBArray(camera, size=tuple(conf["resolution"]))
 
# allow the camera to warmup, then initialize the average frame, last
# uploaded timestamp, and frame motion counter
print("[INFO] warming up...")
time.sleep(conf["camera_warmup_time"])
avg = None
lastUploaded = datetime.datetime.now()
motionCounter = 0
frameCount = 0
print("[INFO] done warming up...")


# capture frames from the camera
for f in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # grab the raw NumPy array representing the image and initialize
    # the timestamp and occupied/unoccupied text
    frame = f.array
    timestamp = datetime.datetime.now()
    text = "Unoccupied"

    if (timestamp - mqtt_timeout).seconds > 30:
        mqtt_timeout = timestamp
        mqtt_client.publish("homeassistant/binary_sensor/picam/heatbeat", '')
                        
    # resize the frame, convert it to grayscale, and blur it
    #frame = imutils.resize(frame, width=500)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (21, 21), 0)
                                       
    # if the average frame is None, initialize it
    if avg is None:
        print("[INFO] starting background model...")
        avg = gray.copy().astype("float")
        rawCapture.truncate(0)
        continue
                                                                                
    # accumulate the weighted average between the current frame and
    # previous frames, then compute the difference between the current
    # frame and running average
    cv2.accumulateWeighted(gray, avg, 0.5)
    frameDelta = cv2.absdiff(gray, cv2.convertScaleAbs(avg))

    # threshold the delta image, dilate the thresholded image to fill
    # in holes, then find contours on thresholded image
    thresh = cv2.threshold(frameDelta, conf["delta_thresh"], 255,
        cv2.THRESH_BINARY)[1]
    thresh = cv2.dilate(thresh, None, iterations=2)
    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
    cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if imutils.is_cv2() else cnts[1]

    if len(cnts) == 0 and mqtt_last_state != "OFF":
        mqtt_client.publish("homeassistant/binary_sensor/picam/state", "OFF", retain=True)
        mqtt_last_state = "OFF"
                    
    # loop over the contours
    for c in cnts:
        # if the contour is too small, ignore it
        if cv2.contourArea(c) < conf["min_area"]:
            continue

                                                            
        # compute the bounding box for the contour, draw it on the frame,
        # and update the text
        (x, y, w, h) = cv2.boundingRect(c)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        text = "Occupied"
                                                                                                    
        # draw the text and timestamp on the frame
        ts = timestamp.strftime("%m-%d-%y-%H:%M:%S%f")
        cv2.putText(frame, "Room Status: {}".format(text), (10, 20),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cv2.putText(frame, ts, (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX,
            0.35, (0, 0, 255), 1)

        # check to see if the room is occupied
        if text == "Occupied":

            # check to see if enough time has passed between uploads
            if (timestamp - lastUploaded).seconds >= conf["min_upload_seconds"]:
                # increment the motion counter
                motionCounter += 1
                # check to see if the number of frames with consistent motion is
                # high enough
                if motionCounter >= conf["min_motion_frames"]:
                    frameCount += 1
                    # check to see if dropbox sohuld be used
                    # write the image to temporary file
                    t = TempImage(time_stamp=ts,frame_count=frameCount)
                    cv2.imwrite(t.path, frame)
                    # upload the image to Dropbox and cleanup the tempory image
                    print("[UPLOAD] {}".format(ts))
                    #copy(t.path,"/home/sean/images/")
                    os.system('cp "%s" "%s"' % (t.path, conf["image_copy_to_location"]))
                    t.cleanup()

                    # update the last uploaded timestamp and reset the motion
                    # counter
                    lastUploaded = timestamp

            if mqtt_last_state != "ON":
                mqtt_client.publish("homeassistant/binary_sensor/picam/state", "ON", retain=True)
                mqtt_last_state = "ON"

        #otherwise, the room is not occupied
        else:
            motionCounter = 0
            if mqtt_last_state != "OFF":
                mqtt_client.publish("homeassistant/binary_sensor/picam/state", "OFF", retain=True)
                mqtt_last_state = "OFF"


    # check to see if the frames should be displayed to screen
    if conf["show_video"]:
        # display the security feed
        cv2.imshow("Security Feed", frame)
        key = cv2.waitKey(1) & 0xFF
               
        # if the `q` key is pressed, break from the lop
        if key == ord("q"):
            break
                                             
    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)
