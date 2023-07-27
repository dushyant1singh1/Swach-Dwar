from tensorflow.python.keras.backend import dtype
from tensorflow.python.ops.control_flow_ops import case_v2
import RPi.GPIO as GPIO
import time
from gpiozero import Buzzer
import Adafruit_CharLCD as LCD
from mfrc522 import SimpleMFRC522
from tensorflow.keras.applications.mobilenet_v2 import preprocess_input
from tensorflow.keras.preprocessing.image import img_to_array
from tensorflow.keras.models import load_model
from imutils.video import VideoStream
import numpy as np
import imutils
import cv2
import os
from mfrc522 import SimpleMFRC522
from smbus2 import SMBus
from mlx90614 import MLX90614
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# Defining the GPIO Pins for LCD
lcd_rs = 16
lcd_en = 24
lcd_d4 = 23
lcd_d5 = 17
lcd_d6 = 18
lcd_d7 = 22
lcd_backlight = 2

lcd_columns = 16
lcd_rows = 2

lcd = LCD.Adafruit_CharLCD(lcd_rs, lcd_en, lcd_d4, lcd_d5, lcd_d6, lcd_d7, lcd_columns, lcd_rows, lcd_backlight)

# Defining the GPIO Pins for UltraSonic Sensor
echo = 20
trig = 21
GPIO.setup(trig, GPIO.OUT)
GPIO.setup(echo, GPIO.IN)

# Defining the GPIO Pins for Servo Motor


# Defining the GPIO Pins for Infrared Sensor
infrared = 26
GPIO.setup(infrared, GPIO.IN)

# Defining the GPIO Pins for LED
led = 3
GPIO.setup(led, GPIO.OUT)

# Defining the GPIO Pins for Buzzer
buzzer = Buzzer(2)

lcd.message('  SWACHH-DWAAR  ')
time.sleep(3)
lcd.clear()


lcd.message('Loading ML Model')
prototxtPath= r"/home/pi/Desktop/new_model/facemask_detection/face_detector/deploy.prototxt"
weightsPath=r"/home/pi/Desktop/new_model/facemask_detection/face_detector/res10_300x300_ssd_iter_140000.caffemodel"
faceNet = cv2.dnn.readNet(prototxtPath, weightsPath)

# load the face mask detector model from disk
maskNet = load_model("mask_detector.model")
lcd.clear()


# ******* DEFINING ALL KINDS OF HELPING FUNCTIONS *******
def measuring_temp():
    bus = SMBus(1)
    sensor = MLX90614(bus, address = 0x5A)
    temp = sensor.get_object_1()
    return temp

def ultrasonic_dist():
    GPIO.output(trigger,True)

    time.sleep(0.00001)
    GPIO.output(trigger,False)

    start=time.time()
    end=time.time()

    while GPIO.input(echo)==0:
        start=time.time()

    while GPIO.input(echo)==1:
        end=time.time()

    total=end-start
    distance=(total*31400)/2
    
    return distance
    

def infrared_detect():
    try:
        while True:
            if GPIO.input(infrared):
                temp = measuring_temp()
                if temp:
                    return temp
                
    except KeyboardInterrupt:
        GPIO.cleanup()

def buzzer_live():
    buzzer.on()
    time.sleep(1)
    buzzer.off()


def detect_and_predict_mask(frame,faceNet,maskNet):
    # saving the dimensions of the frame
    (h,w) = frame.shape[:2]
    blob = cv2.dnn.blobFromImage(frame, 1.0, (224,224), (104.0 ,177.0 , 123.0))

    # pass the blob through the network and obtain the face detections 
    faceNet.setInput(blob)
    detections = faceNet.forward()
    print(detections.shape)
    # Initializing list of faces and the list of predictions from our face mask model
    faces = []
    preds = []

    # Iterating over all the detections
    for i in range(0, detections.shape[2]):
        # Extracting the confidence mapped with the detection
        confidence = detections[0, 0, i, 2]

        # Filtering out all weak detections be ensuring the confidence is greater
        # than the minimum confidence

        if (confidence > 0.5):
            box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
            (startX, startY, endX, endY) = box.astype("int")
        	# Confirming that all the boxes fall withing the dimension of the frame
            (startX, startY) = (max(0, startX), max(0, startY))
            (endX, endY) = (min(w - 1, endX), min(h - 1, endY))

         	# Extract the face ROI
         	# Resizing to 224, 224
            face = frame[startY:endY, startX:endX]
            face = cv2.resize(face, (224,224))
            face = img_to_array(face)
            face = preprocess_input(face)

            # add the face and bounding boxes to their respective lists
            faces.append(face)
    if len(faces) > 0:
        print('h')
        faces = np.array(faces, dtype='float32')
        preds = maskNet.predict(faces, batch_size = 32)
    return preds



def RFID_sensor():
    # Defining the Presets for RFID
    reader = SimpleMFRC522()
     
    id, text = reader.read()
    return (id,text)

def servo_live():
    servo = 13
    GPIO.setup(servo, GPIO.OUT)
    p=GPIO.PWM(13,50)
    p.start(2.5)
    p.ChangeDutyCycle(8.5)
    time.sleep(1)
    p.ChangeDutyCycle(1.5)
    time.sleep(5)
    p.ChangeDutyCycle(8.5)
    time.sleep(1)
    GPIO.cleanup(13)

def mask_detection_video():
    avg_pred=[0,0,0,0,0]
    mask_detected=0
    i=0
    
    while True:
        # grabbing the frame from the threaded video stream and resizing it
        if i==0:
            lcd.message('    Starting\n  Video Stream')
            vs=VideoStream(src=0).start()
            lcd.clear()
        frame = vs.read()
        frame = imutils.resize(frame, width = 500)

        # detect faces in the frame and determine if they are wearing a face mask or not
        preds = detect_and_predict_mask(frame, faceNet, maskNet)

        # for every prediction checking for accuracy
        for pred in preds:
            (mask, withoutmask) = pred

            # deleting the oldest frame prediction
            del avg_pred[0]
            avg_pred.append(mask)
            print(mask)
        if sum(avg_pred)/5 > .5:
            mask_detected=1
            vs.stop()
            return mask_detected
        else:
            if i>5:
                print(i)
                lcd.message('Purchase Mask')
                vs.stop()
                i=0
                text, id1 = RFID_sensor()
                lcd.clear()
            
                if id1:
                    lcd.message('mask unit')
                    servo_live()
                    lcd.clear()
                    lcd.message('mask purchased')
                    time.sleep(3)
                    lcd.clear()
                    return True
        i+=1
while True:
    lcd.message('Signal Awaited..')

    # status is a variable for checking infrared detection
    temp = infrared_detect()
    lcd.clear()
    lcd.message('Object Detected')
    time.sleep(2)
    lcd.clear()
    s = 'Temperature ' + str(temp) + '°F'
    lcd.message(s)
    time.sleep(1)
    lcd.clear()

    detected = 0

    if int(temp)>101.0:
        lcd.message('You have Fever\nNot Allowed')
        buzzer_live()
        time.sleep(1)
        lcd.clear()
    else:
        lcd.message('Face Mask\nDetection')
        time.sleep(3)
        lcd.clear()
        lcd.message('Plz go to Checkbox')
        time.sleep(3)
        lcd.clear()

        # face mask detection
        detected = mask_detection_video()
        if detected: 
            lcd.message('Sanitize YourSelf')
            time.sleep(5)
            lcd.clear()
            lcd.message('Gates Are Opening')
            buzzer_live()
            buzzer_live()
            buzzer_live()
            lcd.clear()
    
    dist = ultrasonic_dist()
    if dist > 11:
        
        lcd.message('Refill Santizer\n Dispenser')
        buzzer_live()
        time.sleep(1)
        buzzer_live()
        time.sleep(1)




