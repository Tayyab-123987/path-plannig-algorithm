import cv2
import numpy as np
import imutils
import dronekit
import math
import time
from dronekit_sitl import SITL

#capturing the video
cap = cv2.VideoCapture(0)
#config the window dimensions
cap.set(3,640)
cap.set(4,480)
# GIVING the resolution of A result window
resx = 640 
resy = 480
#following is to save the video
result = cv2.VideoWriter('video.avi',cv2.VideoWriter_fourcc(*'MJPG'), 10,(resx,resy), )
#CENTER COORDINATES of CAMERA 
centx = resx/2 
centy = resy/2 
centre_camera = [centx, centy]
#initialising SITL
sitl = SITL()
sitl.download('copter', '3.3', verbose = True)
sitl_args = ['-I0', '--model', 'quad']
sitl.launch(sitl_args, await_ready = True, restart = True)
connection_string = 'tcp:127.0.0.1:5760'

#connect to the drone
vehicle = dronekit.connect(connection_string, baud=57600)

#check if vehicle is armable. This ensures home location is set
while not vehicle.is_armable:
    print("Waiting for vehicle to initialise...")
    time.sleep(10)
print("Vehicle Initialized!")

#set high values for minimum distances between marker and camera
min_dist_blue = 100000000000
min_dist_red = 100000000000
gps_blue=[]
gps_red=[]
total_time=1
while (cap.isOpened()):

    #lon,lat= vehicle.location.global_frame
    latitude = vehicle.location.global_frame.lat
    longitude = vehicle.location.global_frame.lon

    start_time = time.time()

    #reading frame by frame
    ret, frame = cap.read()
    
    if ret == False:
        break

    #converting the frame to hsv
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV )

    #Following values are determined experimentally using trackbars

    #for Blue: (94,140,115,255,0,255)
    blue_min = np.array([94,115,0])
    blue_max = np.array([140, 255, 255])
    #creating a mask for colour blue
    bluemask = cv2.inRange(hsv_frame, blue_min, blue_max)
    blue = cv2.bitwise_and(frame, frame, mask=bluemask)

    #for red: (0,22,73,255,0,255)
    red_min = np.array([0, 73, 0])
    red_max = np.array([22,255,255])
    #creating a mask for colour red
    redmask = cv2.inRange(hsv_frame, red_min, red_max)
    red = cv2.bitwise_and(frame, frame, mask=redmask)

    #creating and grabbing blue contours
    blue_contour = cv2.findContours(bluemask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    blue_contour = imutils.grab_contours(blue_contour)

    #creating and grabbing red contours
    red_contour = cv2.findContours(redmask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    red_contour = imutils.grab_contours(red_contour)
    for contour in blue_contour:
        area = cv2.contourArea(contour)
        if area > 750:
            x,y,w,h = cv2.boundingRect(contour)
            cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
            Cx = (x+x+w)/2
            Cy = (y+y+h)/2

            #or we can use moments to find centres as follows
            '''M = cv2.moments(contour)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])'''
            centre_BMarker = [Cx, Cy]

            #finding distance between centre of marker and centre of camera
            dist = math.dist(centre_BMarker, centre_camera)

            #following condition saves the minimum distance uptill now in the variable min distance along with the gps coordinates
            if dist<min_dist_blue:
                min_dist_blue = dist
                gps_blue = [latitude, longitude]

    for contour in red_contour:
        area = cv2.contourArea(contour)
        if area > 750:
            x,y,w,h = cv2.boundingRect(contour)
            cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,255),2)
            Cx = (x+x+w)/2
            Cy = (y+y+h)/2
            
            #or we can use moments to find centres as follows
            '''M = cv2.moments(contour)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])'''

            centre_RMarker = [Cx, Cy]
            dist = math.dist(centre_RMarker, centre_camera)
            if dist<min_dist_red:
                min_dist_red = dist
                gps_red = [latitude, longitude]
    
    result.write(frame)
    cv2.imshow("result", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    cv2.waitKey(1)
    total_time = time.time() - start_time
#showing gps values of vehicle for which the red and blue markers were least distance apart 
print("The gps coordinates [lat, long] of blue marker are: ",gps_blue)
print("The gps coordinates [lat, long] of red marker are: ",gps_red)

result.release()
cap.release()
cv2.destroyAllWindows()
fps = 1/total_time
print("THE FPS OF THE FLIGHT IS ",fps)