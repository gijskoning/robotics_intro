import cv2
import numpy as np
import time 
import serial

#open video
cap = cv2.VideoCapture(1)

if not cap.isOpened():
    print("camera not working, is the port correct?")
    quit()
# setup blob detection
params = cv2.SimpleBlobDetector_Params()
params.filterByColor = True
params.blobColor = 0 # 0 - 255 extract dark blobs - extract light blobs
detector = cv2.SimpleBlobDetector_create(params) #uses opencv 4.0.0

# setup serial connection
# https://stackoverflow.com/questions/24214643/python-to-automatically-select-serial-ports-for-arduino
ser = serial.Serial(
    port='\\\\.\\COM4',
    baudrate=9600,
    parity=serial.PARITY_ODD)
if ser.isOpen():
    ser.close()
ser.open()
while True:
    ser.write("1")
    time.sleep(1)
    print("check")
ser.close()

while True:
    #time.sleep(0.1)
    ret, frame = cap.read()
    if not ret:
        print("No frame recevied. Exiting...")
        break
    
    #show the live video feed
    if cv2.waitKey(1) == ord('q'):
        break
    # find the object
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    keypoints =  detector.detect(gray)
    #print(keypoints)
    frame_keypoints = cv2.drawKeypoints(frame, keypoints,np.array([]),(0,0,255),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS) 
    if len(keypoints)>=1:
        location = keypoints[0].pt #location in pixels (x,y), (floats)
        #draw mini circle around center of blob
        cv2.circle(frame_keypoints,(int(location[0]),int(location[1])),5,(255,255,0), 1)
    cv2.imshow('Press q to exit', frame_keypoints)

    
cap.release()
cv2.destroyAllWindows()
#if not cap.isOpened():
#    raise IOError("cannot open webcam")

