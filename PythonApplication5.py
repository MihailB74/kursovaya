import cv2
import numpy as np
import imutils
import matplotlib.pyplot as plt
import time



def nothing(x):
    pass
sigma = 200;
t = 0
stopcount = 0
XYSWHarr=np.zeros((2,5))
#XYSarr = [0 for i in range(20)]
kernel = np.ones((5,5), np.uint8)

cap = cv2.VideoCapture(0)

cv2.namedWindow("track",cv2.WINDOW_NORMAL)

cv2.createTrackbar("H","track",0,180,nothing)
cv2.createTrackbar("S","track",0,255,nothing)
cv2.createTrackbar("V","track",0,255,nothing)

cv2.createTrackbar("HL","track",0,180,nothing)
cv2.createTrackbar("SL","track",0,255,nothing)
cv2.createTrackbar("VL","track",0,255,nothing)

while True:

    ret,frame = cap.read()
    #frame = cv2.bilateralFilter(frame, 9, 75, 75)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    h = cv2.getTrackbarPos("H","track")
    s = cv2.getTrackbarPos("S","track")
    v = cv2.getTrackbarPos("V","track")

    hl = cv2.getTrackbarPos("HL","track")
    sl = cv2.getTrackbarPos("SL","track")
    vl = cv2.getTrackbarPos("VL","track")
    
    lower = np.array([hl,sl,vl])
    upper = np.array([h,s,v])
    mask = cv2.inRange(hsv,lower,upper)
    #res = cv2.bitwise_and(frame,frame, mask=mask)
    opening = cv2.morphologyEx(mask,cv2.MORPH_OPEN, kernel)
    closing = cv2.morphologyEx(opening,cv2.MORPH_CLOSE, kernel)
    
    contours, h = cv2.findContours(closing, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    contours = sorted(contours, key = cv2.contourArea, reverse = True)
    x,y,w,h = 0,0,0,0
    for x in range(len(contours)):
        area = cv2.contourArea(contours[0])
        if area > 300:
            x,y,w,h = cv2.boundingRect(contours[0])
            frame = cv2.rectangle(frame, (x,y), (x+w, y+h), (0,255,0), 2)
            frame = cv2.rectangle(frame,(x,y), (x+100, y-25), (0,0,0), -1)
            cv2.putText(frame,"Model",(x,y),cv2.FONT_HERSHEY_SIMPLEX,1.0,(255,255,255),2)

            frame = cv2.drawContours(frame, contours, -1, (255,0,0), 2)
            frame = cv2.circle(frame, (x+(w//2),y+(h//2)),8,(0,255,0),2)

    time.sleep(0.05)
    t+=0.05
    if ((t//5)<0.001):
        stopcount+=1

    if (t==30.0):
        for i in range(len(XYSWHarr)):
            XYSWHarr[i,0]=x
            XYSWHarr[i,1]=y
            XYSWHarr[i,2]=w*h
            XYSWHarr[i,3]=w
            XYSWHarr[i,4]=h
    elif ((t>300)&((t//5)<0.001)&((w*h)-XYSWHarr[0,2]<0.1*XYSWHarr[0,2])):
        stopcount = 0
        if (((x+w/2)-((XYSWHarr[1,0])+(XYSWHarr[1,3]/2)))>XYSWHarr[1,2]*0.1):
            print("ОТКЛОНЕНИЕ")
            break
        else:
            XYSWHarr[1,0]=XYSWHarr[0,0]
            XYSWHarr[1,1]=XYSWHarr[0,1]
            XYSWHarr[1,2]=XYSWHarr[0,2]
            XYSWHarr[1,3]=XYSWHarr[0,3]
            XYSWHarr[1,4]=XYSWHarr[0,4]
            XYSWHarr[0,0]=x
            XYSWHarr[0,1]=y
            XYSWHarr[0,2]=w*h
            XYSWHarr[0,3]=w
            XYSWHarr[0,4]=h
    elif (stopcount>50):
        print("ОТКЛОНЕНИЕ")
        break
    else:
        continue
        #print(XYSWHarr[1,0],XYSWHarr[1,1],XYSWHarr[1,2],round(t,1),"    ",x+(w//2),y+(h//2),w,h, "      ", first10x, last10x,(first10x-last10x)/10)
    cv2.imshow("mask", mask)
    cv2.imshow("close", closing)
    cv2.imshow("frame", frame)

    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        break
cv2.destroyAllWindows()