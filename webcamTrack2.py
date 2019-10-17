Python 3.6.4 (v3.6.4:d48eceb, Dec 19 2017, 06:54:40) [MSC v.1900 64 bit (AMD64)] on win32
Type "copyright", "credits" or "license()" for more information.
>>> ## Please make sure your USB camera is connected to the RPi before running this program.
from classa import WebcamVideoStream
from classa import FPS
import RPi.GPIO as GPIO
import time
import cv2
import numpy as np
from threading import Thread
import socket

#capture = cv2.VideoCapture(0)
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(24,GPIO.OUT)
GPIO.setup(23,GPIO.OUT)
GPIO.setup(22,GPIO.OUT)
GPIO.setup(27,GPIO.OUT)
GPIO.setup(4,GPIO.OUT)
GPIO.output(4,GPIO.HIGH)
GPIO.setup(5, GPIO.OUT)
GPIO.setup(6,GPIO.OUT)
GPIO.setup(13,GPIO.OUT)
GPIO.setup(19,GPIO.OUT)
GPIO.output(19,GPIO.HIGH)

kp=0.125
kd=0.00625
flag=True


error=0.0
lasterror=0.0

r=GPIO.PWM(13,255)
g=GPIO.PWM(6,255)
b=GPIO.PWM(5,255)
r.setup(255)
b.setup(255)
g.setup(255)

redlower=(0,20,100)
redupper=(20,70,256)


PWMRB=GPIO.PWM(24,255)
PWMRF=GPIO.PWM(23,255)
PWMLF=GPIO.PWM(27,255)
PWMLB=GPIO.PWM(22,255)
PWMRB.start(0)
PWMRF.start(0)
PWMLF.start(0)
PWMLB.start(0)
vs = WebcamVideoStream(src=0).start()
program_running = True
ack = 0

dict1={'redlower':(00,00,140), 'redupper':(60,60,255),'bluelower':(160,50,40),'blueupper':(230,100,70), 'greenlower':(20,140,10),'greenupper':(70,220,80),'pinklower':(100,50,200), 'pinkupper':(220,100,255),'yellowlower':(0,160,160),'yellowupper':(40,230,230),'violetlower':(130,10,60),'violetupper':(200,60,100),'blacklower':(0,0,0),'blackupper':(30,30,30)}
tup1=['redlower','redupper','bluelower','blueupper','greenlower','greenupper','pinklower','pinkupper','yellowlower','yellowupper','violetlower','violetupper','blacklower','blackupper']
colors={'red':(0,0,255),'blue':(255,0,0),'green':(0,255,0),'pink':(255,0,255),'yellow':(0,255,255),'violet':(130,0,75),'black':(255,255,255)}
tup2=['red','blue','green','pink','yellow','violet','black']


##def send_ack():
##    
##    s1 = socket.socket()
##    port = 3234
##    s1.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
##    s1.bind('', port))
##    s1.listen(5)
##    while program_running:
##        c, addr = s1.accept()
##        c.send(ack)
##   	c.close()

#def checkpointlight(t):
#    red = 255 - t[0]
#    green = 255 - t[1]
#    blue = 255 - t[2]
#    r.ChangeDutyCycle(red)
#    g.ChangeDutyCycle(green)
#    b.ChangeDutyCycle(blue)
#    time.sleep(3)
#    r.ChangeDutyCycle(255)
#    g.ChangeDutyCycle(255)
#    b.ChangeDutyCycle(255)

def checkpoint():
    capture = vs.read()
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
            print "color",colors[tup2[i]]
	    tup2.remove(tup2[i])
            break
	else:
	    flag = False
    return colors[tup2[i]]

                    
def find_missing_laser():
    centre = find_laser()
    if centre is None:
	PWMRF = ChangeDutyCycle(15)
	PWMRB = ChangeDutyCycle(0)
	PWMLB = ChangeDutyCycle(15)
	PWMLF = ChangeDutyCycle(0) 
    while centre is None:	
	centre = find_laser()
    PWMLF = PWMRB = PWMLB = PWMRF = ChangeDutyCycle(0)		
    ack = 1
    return centre


##def send_colour(colour):
##    s2 = socket.socket()
##    port2 = 3233
##    s2.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR)
##    s2.bind()
##    s2.listen(5)
##    while t<2:
##	c2, addr2 = s2.accept()
##	c2.send(str(colour))
##	c2.close


def find_laser():

    frame = vs.read()
      ##rgb to hsv,hue,brightness,shade
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    mask=cv2.inRange(hsv,np.array(redlower),np.array(redupper))
    mask=cv2.erode(mask,None,iterations=2)
    ##erode-removes external noise and makes it smooth line
    mask=cv2.dilate(mask,None,iterations=2)
      #removes internal noise
    

##Contours returns close object, list of pixels
cnts=cv2.findContours(mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
    centre=None
    if len(cnts)>0:
        #finding the centre of all contours,
        c=max(cnts,key=cv2.contourArea)
        ((x,y),radius)=cv2.minEnclosingCircle(c)
        moments=cv2.moments(c)
        if moments["m00"] > 0:
            centre=int(moments["m10"]/moments["m00"]),int(moments["m01"]/moments["m00"])
        else:
            centre = int(x),int(y)
	ack = 1
      
    else:
	ack = 0
      #bot is still
    return centre

        
#    Thread(target = send_ack, args = ()).start()     

while cv2.waitKey(1) != 27:

    centre = find_laser()    
    if centre is not None:
        (x, y) = centre

##        error1 = 240 -y
##        kp1=0.083
##        basespeed = 50+kp1*error1
        error=320-x
       # print error
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

    
PWMR.stop()
PWMR1.stop()
PWML.stop()
PWML1.stop()
GPIO.cleanup()
cv2.destroyAllWindows()

