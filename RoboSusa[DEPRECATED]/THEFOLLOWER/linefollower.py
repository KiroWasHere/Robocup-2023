import cv2 as cv
import numpy as np
import time as t
import random as rng
import math
# Load the video
def calculate_motor_speeds(value):
    # Set the maximum speed of the motors
    max_speed = 50
    
    # Calculate the motor speeds based on the value of value
    if value < -20:
        # Turn left
        motor_left = -max_speed
        motor_right = max_speed * (1 - math.sin(math.pi * abs(value) / 150))
    elif value > 20:
        # Turn right
        motor_left = max_speed * (1 - math.sin(math.pi * abs(value) / 150))
        motor_right = -max_speed
    else:
        # Don't turn
        motor_left = max_speed
        motor_right = max_speed
    motor_left = int(motor_left)
    motor_right = int(motor_right)
    return motor_left, motor_right



cap = cv.VideoCapture("C:/Users/monym/Desktop/ev3OpenCv/VideosTest/testgg.mp4")

volteDivisione = 8
minArea = 0 / volteDivisione


while True:
    ret, frame = cap.read()
    original = cap.read()[1]
    if not ret:
        cap.set(cv.CAP_PROP_POS_FRAMES, 0)
    if frame is None: # Check if the frame is None
        break
    frame = cv.resize(frame, (300, 150))

    # prendo solo la linea in range nero
    # prima di farlo pero converto il frame in HSV
    nLower = np.array([0, 0, 0])
    nHigher = np.array([360, 360, 50])
    mask = cv.cvtColor(frame, cv.COLOR_RGB2HSV)
    mask = cv.inRange(mask, nLower, nHigher)

    # prendo le dimensione del frame della mask
    # divido la maschera in un numero deviso da me di volte verticalmente
    sizeMask = mask.shape
    sizeMH = sizeMask[0]
    sizeMW = sizeMask[1]
    hSq = int(sizeMH / volteDivisione)

    # inizio la divisone dell'immagine in n segmenti
    xSq = 0 # non dovrebbe essere necessario
    ySq = 0
    # hSq è gia definita
    wSq = sizeMW # largo quanto la mask
    pySq = 0
    countourSq = []
    max_contour = None
    max_area = 0
    ftest = frame
    followSquare = True
    for sq in range(volteDivisione):
        max_area = 0
        squareB = mask[pySq:(pySq + hSq), 0:wSq]
        countourSq = cv.findContours(squareB, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        if not countourSq == None:
            for contour in countourSq[0]:
                # Compute the area of the contour
                area = cv.contourArea(contour)
                # If the area is larger than the current maximum area, update the max contour and max area
                if area > max_area:
                    max_contour = contour
                    max_area = area
            M = cv.moments(max_contour)
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
            # offset countourq
            if max_contour is not None:
                areaMaxC = cv.contourArea(max_contour)
                if areaMaxC < minArea:
                    doDraw = False
                else:
                    doDraw = True
                if doDraw:
                    cOff = cx - 150
                    cXx = str(calculate_motor_speeds(cOff))
                    print(cXx + " | " + str(cOff))
                    cv.drawContours(ftest, [max_contour], -1, (0, 125, 0), 3, offset=(0, pySq))
                    if followSquare:
                        cv.circle(ftest, (cx, cy + pySq), 7, (0, 255, 0), -1)
                        cv.putText(ftest, cXx, (cx, cy + pySq + 30), cv.FONT_HERSHEY_SIMPLEX, 0.5,(125, 255, 0), 2, cv.LINE_AA)
                        followSquare = False
                    else:
                        cv.circle(ftest, (cx, cy + pySq), 7, (0, 0, 255), -1)
                cv.imshow("sqr", squareB)
                cv.imshow("ftest", ftest)
                
        pySq = pySq + hSq
    

    # mostro viste
    cv.imshow("mask", mask)
    cv.imshow("original", original)
    cv.imshow("main", frame)
    if cv.waitKey(50) & 0xFF == ord('r'):
        break
cap.release()
cv.destroyAllWindows()
