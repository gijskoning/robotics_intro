import cv2
import numpy as np
import time #only to 'lower the framerate'


#open video
cap = cv2.VideoCapture(1)

if not cap.isOpened():
    print("camera not working, is the port correct?")
    quit()


# setup blob detection
params = cv2.SimpleBlobDetector_Params()
params.filterByColor = True
params.blobColor = 0 # 0 - 255 filter light - dark colors
detector = cv2.SimpleBlobDetector_create(params) #uses opencv 4.0.0

while True:
    time.sleep(0.1)
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

    cv2.imshow('Press q to exit', frame_keypoints)
    #print(keypoints) # not working

cap.release()
cv2.destroyAllWindows()
#if not cap.isOpened():
#    raise IOError("cannot open webcam")

