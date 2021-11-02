from adafruit_servokit import ServoKit
import cv2
import jetson.inference
import jetson.utils
import time
import numpy as np
import threading
import RPi.GPIO as GPIO
import _thread
import serial 
import os
from ctypes import *                                               # Import libraries
import math
import random
import darknet

GPIO.cleanup()
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.setup(11,GPIO.OUT)
GPIO.setup(13,GPIO.OUT)
GPIO.setup(15,GPIO.OUT)
GPIO.output(11,True)
GPIO.output(13,True)

item = ""
top = 0
bottom = 0
left = 0
right = 0
stopBot = 0
controlSend = 0
pick = 0
pos_servo0=0
pos_servo1=0
pos_servo2=0
pos_servo3=0
stoptimer=1
stopfunction=0
stop_arm=0
val = 1
run = 0
time.sleep(5)
#try:
   # arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=10)
    #print("Try running!************")
#except Exception as e:
    #print("Exception : ",e)
arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=10)

def servo_run():
    global pos_servo0
    global pos_servo1
    global pos_servo2
    global pos_servo3
    
    myKit=ServoKit(channels=16)

    myKit.servo[0].angle=18
    #time.sleep(3)
    # myKit.servo[1].angle=0
    # time.sleep(3)
    # myKit.servo[2].angle=30
    # time.sleep(8)
    myKit.servo[3].angle=0
   
    for i in range(0,20,2):
        myKit.servo[2].angle=i
        #print("clockwise 1")
        #print(i)
        time.sleep(0.05)
        pos_servo2=i
         
    for i in range(15,20,1):
        myKit.servo[1].angle = i
        #print("clockwise 1")
        #print(i)
        time.sleep(0.05)
        pos_servo1 = i

    pos_servo3=0
    for pos_servo3 in range(pos_servo3,23,1):
        myKit.servo[3].angle = pos_servo3
        # print("clockwise 0")
        # print(pos_servo1)
        time.sleep(0.05)


def servo_initialize():
    global pos_servo0
    global pos_servo1
    global pos_servo2
    global pos_servo3
    
    myKit=ServoKit(channels=16)
    if(pos_servo2<5):
        for pos_servo2 in range(pos_servo2,5,1):
            myKit.servo[2].angle=pos_servo2
            time.sleep(0.05)

    elif(pos_servo2>5):
        for pos_servo2 in range(pos_servo2,5,-1):
            myKit.servo[2].angle=pos_servo2
            time.sleep(0.05)

    if(pos_servo3<25):
        for pos_servo3 in range(pos_servo3,25,1):
            myKit.servo[3].angle=pos_servo3
            time.sleep(0.05)

    elif(pos_servo3>25):
        for pos_servo3 in range(pos_servo3,25,-1):
            myKit.servo[3].angle=pos_servo3
            time.sleep(0.05)

    if(pos_servo1<30):
        for pos_servo1 in range(pos_servo1,30,1):
            myKit.servo[1].angle=pos_servo1
            time.sleep(0.05)

    elif(pos_servo1>30):
        for pos_servo1 in range(pos_servo1,30,-1):
            myKit.servo[1].angle=pos_servo1
            time.sleep(0.05)

def lock_on():
    global item
    global top
    global bottom
    global left
    global right
    global pos_servo0
    global pos_servo1
    global pos_servo2
    global pos_servo3
    global stopfunction
    global stoptimer
    global stop_arm 
    toplock = 0
    bottomlock = 0
    rightlock = 0
    leftlock = 0
    stop_arm = 0
    count = 0
    myKit=ServoKit(channels=16)
    while (stop_arm == 0): 
        
        if (stopfunction==1):
            stopfunction=0
            break

        print("Lock on Running, Stop Fn = %i"%(stopfunction))
        #print("stop fn = ",stopfunction)
        #print("lock on running")
        print(item,top,left,bottom,right)

        cx=0 #x co-ordinate of centroid of Redbox 
        cy=0  #y co-ordinate of centroid of Redbox
        w=0 #width of object
        h=0 #height of object
        area=0

        w=right-left
        h=bottom-top

        cx=left+w/2
        cy=top+h/2

        Point_center = (int((left + right)/2), int((top+ bottom)/2))

        cv2.circle(img, (cx,cy) , 5, (255, 0, 0), 3) #Centre of Joy's (object)
        cv2.circle(img, Point_center , 5, (255, 0, 0), 3) #Centre of Object
        cv2.circle(img, (640,360) , 5, (255, 255, 0), 3) #Centre of Camera

        print("Object Centre is (%i, %i) "%(cx, cy))
        if(toplock!=top or bottomlock!=bottom or rightlock!=right or leftlock!=left):
            if(cx>645): #645 for 720p #325 for 360p
                if(pos_servo3>0):
                    pos_servo3=pos_servo3-1
                    myKit.servo[3].angle=pos_servo3

            elif(cx<635): #635 for 720p #315 for 360p
                if(pos_servo3<180):
                    pos_servo3=pos_servo3+1
                    myKit.servo[3].angle=pos_servo3

            if(cy>365): #365 for 720p #185 for 360p
                if(pos_servo1>0):
                    pos_servo1=pos_servo1-1
                    myKit.servo[1].angle=pos_servo1

            elif(cy<355): #355 for 720p #175 for 360p
                if(pos_servo1<90):
                    pos_servo1=pos_servo1+1
                    myKit.servo[1].angle=pos_servo1

            area=w*h

            print(area)

            if(area<180000): #180000 for 720p 90000 for 360p
                pos_servo1=pos_servo1+1
                myKit.servo[1].angle=pos_servo1
                pos_servo2=pos_servo2+1
                myKit.servo[2].angle=pos_servo2

            elif(item=="Redbox" and area>=180000):
                    stop_arm=1
                    stoptimer=1
               
                
        leftlock=left
        rightlock=right
        toplock=top
        bottomlock=bottom
        time.sleep(0.05)


def pick_up():
    global pos_servo0
    global pos_servo1
    global pos_servo2
    global pos_servo3
    myKit=ServoKit(channels=16)

    for i in range(0,90,2):
        myKit.servo[0].angle=i
        #print("clockwise 1")
        #print(i)
        time.sleep(0.05)
        pos_servo0=i
         
    for i in range(0,7,1):
        myKit.servo[1].angle=i+pos_servo1
        #print("clockwise 1")
        #print(i)
        time.sleep(0.05)
        pos_servo1=i+pos_servo1

    time.sleep(0.5)

    for i in range(0,6,1):
        myKit.servo[2].angle=i+pos_servo2
        #print("clockwise 1")
        #print(i)
        time.sleep(0.05)
        pos_servo2=i+pos_servo2
    time.sleep(0.5)
    
    for i in range(90,0,-2):
        myKit.servo[0].angle=i
        #print("clockwise 1")
        #print(i)
        time.sleep(0.05)
        pos_servo0=i

    for pos_servo2 in range(pos_servo2,20,-1):
        myKit.servo[2].angle=pos_servo2
        #print("clockwise 1")
        #print(i)
        time.sleep(0.05)

    for pos_servo1 in range(pos_servo1,30,-1):
        myKit.servo[1].angle=pos_servo1
        #print("clockwise 1")
        #print(i)
        time.sleep(0.05)

def main1() :
    global val
    global Left
    global Right
    global Front
    global Back
    global stopBot
    while True:
        
        while arduino.in_waiting:
        
            print("thread1 running....................... \n")
            for i in range (1,2,1):
                
                data = arduino.readline()
                #if data:
                #print(data)
                numbers = data.decode('utf-8')
                arr = []
                for word in numbers.split():
                    if word.isdigit():
                        arr.append(int(word))
                

                if (len(arr)==0):
                    continue
                distance = arr[0]
                
                if (val == 1):
                    stopBot = distance
                    val = 2
                elif (val == 2):
                    Front = distance
                    val = 1
                    
                
                print("stopBot value: ",stopBot)
                print("\n")
                #if (arduino.in_waiting > 4):
                    #serial.Serial.reset_input_buffer()
                
            
                

def sendY():
    global controlSend

    if (controlSend==1):
        GPIO.output(11,True)
        #GPIO.output(13,False)
        #GPIO.output(15,False)
    else:
        GPIO.output(11,False)
        #GPIO.output(13,False)
        #GPIO.output(15,False)

def convertBack(x, y, w, h):
    xmin = int(round(x - (w / 2)))
    xmax = int(round(x + (w / 2)))
    ymin = int(round(y - (h / 2)))
    ymax = int(round(y + (h / 2)))
    return xmin, ymin, xmax, ymax


def cvDrawBoxes(detections, img):
    # Colored labels dictionary
    color_dict = {
        'Redbox' : [255, 0, 0]
    }

    xmin = []
    xmax = []
    ymin = []
    ymax = []
    label = 0

    for label, confidence, bbox in detections:
        x, y, w, h = (bbox[0], bbox[1], bbox[2], bbox[3])
        name_tag = label
        for name_key, color_val in color_dict.items():
            if name_key == name_tag:
                color = color_val 
                xmin, ymin, xmax, ymax = convertBack(float(x), float(y), float(w), float(h))
                pt1 = (xmin, ymin)
                pt2 = (xmax, ymax)

                cv2.rectangle(img, pt1, pt2, color, 1)
                #print(pt1)
                #print(pt2)
                #Point_center = (int((xmin + xmax)/2), int((ymin + ymax)/2))
                #print("Centroid",Point_center)
                #cv2.circle(img, Point_center, 5, color, 3)
                cv2.putText(img, label + " [" + confidence + "]", (pt1[0], pt1[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    
    xmax = np.reshape(xmax, (-1, 1))
    xmin = np.reshape(xmin, (-1, 1))
    ymin = np.reshape(ymin, (-1, 1))
    ymax = np.reshape(ymax, (-1, 1))

    return img, xmin, ymin, xmax, ymax

netMain = None
metaMain = None
altNames = None


def object_detection():

    global metaMain, netMain, altNames
    global item, top, bottom, left, right
    timeStamp=time.time()
    fpsFiltered=0

    configPath = "/home/jetbot/Desktop/Autonomous_4Wheel_4DoF_Robotic_Arm_Robot_ANGAAD/darknet/cfg/custom-yolov4-tiny-detector.cfg"                                 
    weightPath = "/home/jetbot/Desktop/Autonomous_4Wheel_4DoF_Robotic_Arm_Robot_ANGAAD/darknet/custom-yolov4-tiny-detector_last_2.weights"                                
    metaPath = "/home/jetbot/Desktop/Autonomous_4Wheel_4DoF_Robotic_Arm_Robot_ANGAAD/darknet/cfg/obj_tiny.data"                                   

    if not os.path.exists(configPath):                             
        raise ValueError("Invalid config path `" +
                         os.path.abspath(configPath)+"`")
    if not os.path.exists(weightPath):
        raise ValueError("Invalid weight path `" +
                         os.path.abspath(weightPath)+"`")
    if not os.path.exists(metaPath):
        raise ValueError("Invalid data file path `" +
                         os.path.abspath(metaPath)+"`")

    if netMain is None:                                             
        netMain = darknet.load_net_custom(configPath.encode( 
            "ascii"), weightPath.encode("ascii"), 0, 1)  
                       # batch size = 1
    if metaMain is None:
        metaMain = darknet.load_meta(metaPath.encode("ascii"))

    if altNames is None:
        try:
            with open(metaPath) as metaFH:
                metaContents = metaFH.read()
                import re
                match = re.search("names *= *(.*)$", metaContents,
                                  re.IGNORECASE | re.MULTILINE)
                if match:
                    result = match.group(1)
                else:
                    result = None
                try:
                    if os.path.exists(result):
                        with open(result) as namesFH:
                            namesList = namesFH.read().strip().split("\n")
                            altNames = [x.strip() for x in namesList]
                except TypeError:
                    pass
        except Exception:
            pass

        network, class_names, class_colors = darknet.load_network(configPath,  metaPath, weightPath, batch_size=1)

    #cap = cv2.VideoCapture(0)                                     
    #cap = cv2.VideoCapture("test2.mp4")                         
    cam = cv2.VideoCapture(0 ,cv2.CAP_V4L)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    font=cv2.FONT_HERSHEY_SIMPLEX

    dispW = 1280                         
    dispH = 720
    #print(dispH)
    #print(dispW)
    # Set out for video writer
    #out = cv2.VideoWriter(                                         
    #    "./Demo/output.avi", cv2.VideoWriter_fourcc(*"MJPG"), 10.0,
    #    (frame_width, frame_height))

    #print("Starting the YOLO loop...")


    darknet_image = darknet.make_image(dispW, dispH, 3) 
    while True:                                                
        prev_time = time.time()
        ret, img = cam.read()                                
        
        if not ret:                                                 
            break
        
        #print("after frame read")
        frame_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)      
        frame_resized = cv2.resize(frame_rgb, (dispW, dispH), interpolation=cv2.INTER_LINEAR)

        darknet.copy_image_from_bytes(darknet_image,frame_resized.tobytes())               

        detections = darknet.detect_image(network, class_names, darknet_image, thresh=0.45)                                                                             
        img, left, top, right, bottom = cvDrawBoxes(detections, frame_resized)              
        img = cv2.cvtColor(img , cv2.COLOR_BGR2RGB)
        #print(1/(time.time()-prev_time))
        cv2.imshow('detCam',img)                                  
        cv2.waitKey(3)
        #out.write(image)                                            
    cam.release()                                                   
    #out.release()
    print(":::Video Write Completed")

    '''
    net=jetson.inference.detectNet(argv=["--model=/home/jetbot/Downloads/jetson-inference/python/training/detection/ssd/models/myModel/ssd-mobilenet.onnx", "--labels=/home/jetbot/Downloads/jetson-inference/python/training/detection/ssd/models/myModel/labels.txt", "--input-blob=input_0", "--output-cvg=scores", "--output-bbox=boxes"], threshold=0.5)
    dispW=1280
    dispH=720
    flip=2

    font=cv2.FONT_HERSHEY_SIMPLEX
    cam=jetson.utils.gstCamera(dispW,dispH,'/dev/video0')
    display=jetson.utils.glDisplay()

    cam=cv2.VideoCapture('/dev/video0')
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, dispW)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, dispH)
    while display.IsOpen():
        img,width,height=cam.CaptureRGBA()
    while True:
        _,img = cam.read()
        height=img.shape[0]
        width=img.shape[1]

        frame=cv2.cvtColor(img,cv2.COLOR_BGR2RGBA).astype(np.float32)
        frame=jetson.utils.cudaFromNumpy(frame)

        detections=net.Detect(frame,width,height)
        for detect in detections:
            #print(detect)
            ID=detect.ClassID
            top=int(detect.Top)
            left=int(detect.Left)
            bottom=int(detect.Bottom)
            right=int(detect.Right)
            item=net.GetClassDesc(ID)
            print("object detection: ",item,top)
            #time.sleep(1)
            cv2.rectangle(img,(left,top),(right,bottom),(255,0,0),2)
            cv2.putText(img,item,(left,top+20),font,.75,(0,0,255),2)
        display.RenderOnce(frame,width,height)
        dt=time.time()-timeStamp
        timeStamp=time.time()
        fps=1/dt
        fpsFiltered=0.9*fpsFiltered+.1*fps
        #print(str(round(fps,1))+' fps')

        cv2.putText(img,str(round(fpsFiltered,1))+' fps',(0,30),font,1,(0,0,255),2)
        cv2.imshow('detCam',img)
        cv2.moveWindow('detCam',0,0)
        if cv2.waitKey(1)==ord('q'):
            break
    cam.release()
    cv2.destroyAllWindows()
    '''


def servo_place():
    global pos_servo0 #claw servo
    global pos_servo1
    global pos_servo2 # 
    global pos_servo3 # base servo (Arm Rotation)
    myKit=ServoKit(channels=16)

    for pos_servo3 in range(pos_servo3,150,2):
        myKit.servo[3].angle=pos_servo3
        print("clockwise 1")
        print(pos_servo3)
        time.sleep(0.05)

    for pos_servo2 in range(pos_servo2,40,2):   
        myKit.servo[2].angle=pos_servo2
        # print("clockwise 2")
        # print(pos_servo2)
        time.sleep(0.05)

    for pos_servo0 in range(pos_servo0,60,1):
        myKit.servo[0].angle=pos_servo0
        # print("clockwise 0")
        # print(pos_servo1)
        time.sleep(0.05)

    for pos_servo0 in range(pos_servo0,0,-2):
        myKit.servo[0].angle=pos_servo0
        # print("clockwise 0")
        # print(pos_servo1)
        time.sleep(0.05)

    for pos_servo2 in range(pos_servo2,5,-2):   
        myKit.servo[2].angle=pos_servo2
        # print("clockwise 2")
        # print(pos_servo2)
        time.sleep(0.05)

    for pos_servo3 in range(pos_servo3,25,-1):
        myKit.servo[3].angle=pos_servo3
        # print("clockwise 0")
        # print(pos_servo1)
        time.sleep(0.05)

def timeexceed():
    global stoptimer
    global stopfunction
    while True:
        timecount=0
        while(stoptimer==0):
            print("*********timer running*******")
            time.sleep(1)
            timecount=timecount+1
            if(timecount>=60):
                stoptimer=1
                stopfunction=1
def check_top():
    global top
    global run
    localtop=top
    time.sleep(0.5)
    if(localtop==top):
        run=0
    else:
        run=1


if __name__=="__main__":
    time.sleep(1)
    thread1 = threading.Thread(target=main1)
    thread2 = threading.Thread(target=object_detection)
    thread3 = threading.Thread(target=timeexceed)
    #thread2 = threading.Thread(target=servo_run)
    thread1.start()
    thread2.start()
    thread3.start()
    servo_run()
    servo_initialize()
    while True:
        #print("controlSend= ",controlSend)
        #print("pick= ",pick)
        #print("StopBot = ",stopBot)

        print("Control Send = %i, Pick = %i, StopBot = %i" %(controlSend, pick, stopBot))

        if (stopBot == 1 and pick == 0):
            #thread2.start()
            print("pick function~~~#####")
            controlSend=0
            sendY()
            stoptimer=0
            stopfunction=0
            lock_on()
            stoptimer=1
            if(stop_arm==1):
                stop_arm=0
                pick_up()
                pick=1
                stopBot=0
            servo_initialize()
            top=0
            controlSend=1
            sendY()
            stoptimer=1
            stopfunction=0
            time.sleep(0.05)

        elif (stopBot == 2 and pick == 1):
            print("place function$$$$$$$$***$$$$$$")
            controlSend = 0
            sendY()
            servo_place()
            servo_initialize()
            controlSend = 1
            sendY()
            stopBot=0
            top=0
            pick=0
            time.sleep(0.05)
            

        elif (top>100 and pick==0):
            print("SEnddddd Function0000000")
            GPIO.output(13,False)
            run=0
            check_top()
            if(run==1):
                GPIO.output(13,True)
                controlSend = 0
                sendY()
                run=0
            else:
                top=0
            GPIO.output(13,True)
            time.sleep(0.05)
            
        else:
            print("else function")
            controlSend = 1
            sendY()
            
