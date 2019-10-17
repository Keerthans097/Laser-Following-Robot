# -*- coding: utf-8 -*-
"""
Created on Mon Oct 22 19:18:28 2018

@author: Keerthan S
"""

## Please make sure your USB camera is connected to the RPi before running this program.
#from classa import WebcamVideoStream
#from classa import FPS
import RPi.GPIO as GPIO
import time
import cv2
import numpy as np
from threading import Thread
import socket

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(17,GPIO.OUT) #in1
GPIO.setup(27,GPIO.OUT) #in2
GPIO.setup(19,GPIO.OUT)
GPIO.setup(26,GPIO.OUT)
GPIO.setup(4,GPIO.OUT) #en1
GPIO.output(4,GPIO.HIGH)

GPIO.setup(13,GPIO.OUT)
GPIO.output(13,GPIO.HIGH)

kp=0.125
kd=0.00625
flag=True
error=0.0
lasterror=0.0

#r=GPIO.PWM(13,255)
#g=GPIO.PWM(6,255)
#b=GPIO.PWM(5,255)


#r.GPIO.setup(255)
#b.setup(255)
#g.setup(255)

redlower=(160,20,20)
redupper=(179,255,255)


PWMRB=GPIO.PWM(27,255)
PWMRF=GPIO.PWM(19,255)
PWMLF=GPIO.PWM(17,255)
PWMLB=GPIO.PWM(26,255)
PWMRB.start(0)
PWMRF.start(0)
PWMLF.start(0)
PWMLB.start(0)
video= cv2.VideoCapture(0)
program_running = True
ack = 0

dict1={'redlower':(00,00,140), 'redupper':(60,60,255),'bluelower':(160,50,40),'blueupper':(230,100,70), 'greenlower':(20,140,10),'greenupper':(70,220,80),'pinklower':(100,50,200), 'pinkupper':(220,100,255),'yellowlower':(0,160,160),'yellowupper':(40,230,230),'violetlower':(130,10,60),'violetupper':(200,60,100),'blacklower':(0,0,0),'blackupper':(30,30,30)}
tup1=['redlower','redupper','bluelower','blueupper','greenlower','greenupper','pinklower','pinkupper','yellowlower','yellowupper','violetlower','violetupper','blacklower','blackupper']
colors={'red':(0,0,255),'blue':(255,0,0),'green':(0,255,0),'pink':(255,0,255),'yellow':(0,255,255),'violet':(130,0,75),'black':(255,255,255)}
tup2=['red','blue','green','pink','yellow','violet','black']



def checkpoint():
    capture = video.read()
    count=0
    flag = False
    n=len(tup1)
    n2 = len(tup2)
    for i in range(0,n2):
        mask=cv2.inRange(frame,np.array(dict1[tup2[i]+'lower']),np.array(dict1[tup2[i]+'upper']))
        mask=cv2.erode(mask,None,iterations=2)
        mask=cv2.dilate(mask,None,iterations=2)
        cnts=cv2.findContours(mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
        if len(cnts)>0:
            flag =True
            print ("color",colors[tup2[i]])
            tup2.remove(tup2[i])
	        
            break
        else:
            flag=False

    return colors[tup2[i]]

                    
def find_missing_laser():
    centre = find_laser()
    if centre is None:
        PWMRF.ChangeDutyCycle(15)
        PWMRB.ChangeDutyCycle(0)
        PWMLB.ChangeDutyCycle(15)
        PWMLF.ChangeDutyCycle(0) 
	
	
	
    while centre is None:
        centre = find_laser()
	
   # PWMLF = PWMRB = PWMLB = PWMRF.ChangeDutyCycle(0)		
    ack = 1
    return centre



def find_laser():

    check, frame= video.read()
   # cv2.imshow("video",frame)
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
  #  cv2.imshow("video",frame)
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    mask=cv2.inRange(hsv,np.array(redlower),np.array(redupper))
    mask=cv2.erode(mask,None,iterations=2)
    mask=cv2.dilate(mask,None,iterations=2)
    cnts=cv2.findContours(mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
    cv2.drawContours(frame,cnts,0,(127,255,0),3)
    
    cv2.imshow('frame',frame)
    cv2.imshow('mask',mask)
    centre=None
    if len(cnts)>0:
        c=max(cnts,key=cv2.contourArea)
        ((x,y),radius)=cv2.minEnclosingCircle(c)
        moments=cv2.moments(c)
        if moments["m00"] > 0:
            centre=int(moments["m10"]/moments["m00"]),int(moments["m01"]/moments["m00"])
        else:
            centre = int(x),int(y)
            ack=1
	         
    else:
        ack=0
	
    return centre

        
#    Thread(target = send_ack, args = ()).start()     

while cv2.waitKey(1)!= 27:

    centre = find_laser()    
    if centre is not None:
        (x, y) = centre

        error=320-x
       
        motorspeed = (kp*error + kd*(error-lasterror))
        lasterror = error
         
        rightmotorspeed =  50-motorspeed
        leftmotorspeed = 50+motorspeed
        PWMRB.ChangeDutyCycle(0)
        PWMLB.ChangeDutyCycle(0)
        PWMRF.ChangeDutyCycle(rightmotorspeed)
        PWMLF.ChangeDutyCycle(leftmotorspeed)
    	
    else:
        find_missing_laser()
	
#
#    
#PWMR.stop()
#PWMR1.stop()
#PWML.stop()
#PWML1.stop()
GPIO.cleanup()
video.release()
cv2.destroyAllWindows()
