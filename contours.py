# -*- coding: utf-8 -*-
"""
Created on Thu Oct 25 08:40:23 2018

@author: Keerthan S
"""
    
import numpy as np
import cv2
cap = cv2.VideoCapture(0)
while(1):
  _, frame = cap.read()
  
  hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
  lower_color = np.array([0, 50, 50])
  upper_color = np.array([60, 255, 255])
  mask = cv2.inRange(hsv, lower_color, upper_color)
  mask = cv2.erode(mask, None, iterations=2)
  mask = cv2.dilate(mask, None, iterations=2)
  res = cv2.bitwise_and(frame, frame, mask=mask)

  cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
  cv2.drawContours(frame, cnts, 0, (127, 255, 0), 3)
  print(cnts)
  cv2.imshow('frame', frame)
  cv2.imshow('mask', mask)
  cv2.imshow('res', res)
 # cv2.imshow('contours', frame)



  k = cv2.waitKey(5) & 0xFF
  if k == 27:
    # print"release"

     break
  cap.release()
  cv2.destroyAllWindows()