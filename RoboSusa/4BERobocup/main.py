#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.
import time as t
import math
# Create your objects here.
ev3 = EV3Brick()
ev3.speaker.set_volume(10)
sensoreSx = ColorSensor(Port.S1)
sensoreDx = ColorSensor(Port.S2)
sensoreVM = ColorSensor(Port.S3)
sensoreDist = UltrasonicSensor(Port.S4)
Watch = StopWatch()
motoreSx = Motor(Port.B)
motoreDx = Motor(Port.C)
motoreSx.control.limits(800, 800, 100)
motoreDx.control.limits(800, 800, 100)
"""
motoreBig = Motor(Port.A)
motoreSmall = Motor(Port.D)
"""
# Threshold Colori
# Nero per ora non usato
rgbNero = [[0, 0, 0], [0, 20, 20]]
# .............
rgbVerde = [[4, 9, 9], [9, 18, 18]]
rgbBianco = [[25, 25, 25], [100, 100, 100]]
rgbSLine = [40, 40, 40]
# Velocità
velF = 50
velB = -40
velBV = -70
velFV = 70
velSLF = 50
velSLB = -50
velSLFmin = 40
velStuck = 70
velOstF = 70
velOstB = 32
camb = 30
biasSx = 5
biasDx = 5
# Angoli
ang10CM = 350
ang1CM = ang10CM / 10
ang180 = 950
ang90 = math.floor((ang180 / 2))
# Potenze e velMinime
velMin = 30
pManov = 500

# Misc
thrVerde = 15
thrDoppioVerde = 10
cmRobot = 15
mCmRobot = math.floor((cmRobot / 2))
mCmRobot = mCmRobot * ang1CM
angTurn = 15
tVerde = 1300
cW = 0.1
valWNero = 0
valWNCheck = 0
valWVerde = 0
valWVCheck = 0
tempoW = 1000
stepVerde = 5
stepNero = 5
bSx = 0
bDx = 0
tStall = 3000
stallThresh = 220
# wait
def wait():
    pass

# Funzione di assegnazione colore
# 0 = Nero, 1 = Bianco, 2 = Verde
def assCol(rgbS, uscS = 0):
    if ((rgbS[0] >= rgbVerde[0][0]) and (rgbS[1] >= rgbVerde[0][1]) and (rgbS[2] >= rgbVerde[0][2])) and ((rgbS[0] <= rgbVerde[1][0]) and (rgbS[1] <= rgbVerde[1][1]) and (rgbS[2] <= rgbVerde[1][2])):
        uscS = 2
    elif ((rgbS[0] > rgbBianco[0][0]) and (rgbS[1] > rgbBianco[0][1]) and (rgbS[2] > rgbBianco[0][2])):
        uscS = 1
    else:
        uscS = 0
    return uscS
# Assegnazione colore single line
def assColSL(rgbS, uscS = 0):
    if ((rgbS[0] > rgbSLine[0]) or (rgbS[1] > rgbSLine[1]) or (rgbS[2] > rgbSLine[2])):
        uscS = 1
    else:
        uscS = 0
    return uscS
# Funzione movimento via controllo dc (s = sinistra, d = destra (Potenze di sx e dx))
def mDc(s, d):
    motoreSx.dc(s)
    motoreDx.dc(d)
# Funzione movimento via controllo angolo (aggiunto)
def mAng(power ,angs, angd, then, waitEnd):
    motoreSx.run_angle(power, angs, then, False)
    motoreDx.run_angle(power, angd, then, waitEnd)
# Funzione movimento doppio verde
def doppioVerde():
    mAng(pManov, ang10CM, ang10CM, Stop.HOLD, True)
    mAng(pManov, ang180, -ang180, Stop.HOLD, True)
    mAng(pManov, ang1CM * 5, ang1CM * 5, Stop.HOLD, True)
# Funzione movimento 90 gradi
def ang90Sx():
    mDc(velBV, velFV)
    t.sleep(tVerde)
def ang90Dx():
    mDc(velFV, velBV)
    t.sleep(tVerde)
# Funzione motore con time
def mTime(ssx, sdx, tsx, tdx, then, wait):
    motoreSx.run_time(ssx, tsx, then, False)
    motoreDx.run_time(sdx, tdx, then, wait)

# Funzione movimento edge
tSx = 0
ttSx = 0
tttSx = 0
tDx = 0
ttDx = 0
tttDx = 0
# Movimenti angolari
def mSx():
    mDc(velB, velF + biasSx)

def mDx():
    mDc(velF, velB - biasDx)

def mStop():
    motoreSx.hold()
    motoreDx.hold()

# Funzione verifica verde, resistuisce bool True se vero
def verGreen(sensore):
    val = 0
    val = sensore
    mDc(velMin, velMin)
    intT = 0
    intTT = 0
    while assCol(val.rgb()) == 2:
        intT += 1
        if assCol(sensoreSx.rgb()) == 2 and assCol(sensoreDx.rgb()) == 2:
            intTT += 1
    if intTT >= thrDoppioVerde:
        doppioVerde()
    elif intT >= thrVerde:
        print(str(intT) + " ||| " + str(thrVerde))
        return True
    else:
        print(str(intT) + " ||| " + str(thrVerde))
        intT = 0
        return False
# verifica del doppio verde
def verDoppioGreen():
    mStop()
    intT = 0
    for i in range(2):
        mAng(pManov, stepVerde, stepVerde, Stop.HOLD, False)
        rgbSx = sensoreSx.rgb()
        rgbDx = sensoreDx.rgb()
        colSx = assCol(rgbSx)
        colDx = assCol(rgbDx)
        if colDx != 2 and colSx != 2:
            intT += 1
    if intT == 0:
        return True
    else:
        return False
        mAng(pManov, stepVerde * -3, stepVerde * -3, Stop.HOLD, True)

def SLineSx():
    mStop()
    temp = 1
    colTemp = 0
    watVerde = Watch.time() + tVerde
    while temp:
        colTemp = assColSL(sensoreSx.rgb())
        if colTemp == 1:
            mDc(velSLF, velSLF - camb)
        else:
            mDc(velSLB, velSLF)
        if Watch.time() >= watVerde:
            temp = 0
    while assCol(sensoreDx.rgb()) != 0:
        mDc(-50, 50)

def SLineDx():
    mStop()
    temp = 1
    colTemp = 0
    watVerde = Watch.time() + tVerde
    while temp:
        colTemp = assColSL(sensoreDx.rgb())
        if colTemp == 1:
            mDc(velSLF - camb, velSLF)
        else:
            mDc(velSLF, velSLB)
        if Watch.time() >= watVerde:
            temp = 0
    while assCol(sensoreSx.rgb()) != 0:
        mDc(50, -50)
wPrima = 0
angPSx = 0
angPDx = 0 
angDSx = 0
angDDx = 0
def stuckFix():
    global wPrima
    global angPSx
    global angPDx
    print("ANGOLI: " + str(motoreSx.angle()) + " | " + str(angPSx))
    if Watch.time() > wPrima + tStall:
        wPrima = Watch.time()
        if ((motoreSx.angle() - angPSx) < stallThresh and (motoreDx.angle() - angPDx) < stallThresh):
            ev3.speaker.beep(100, 100)
            angPSx = motoreSx.angle()
            angPDx = motoreDx.angle()
            # Attivazione dell'anti-stallo
            mDc(velStuck, velStuck)
            t.sleep(0.5)
            while assCol(sensoreSx.rgb()) != 1 and assCol(sensoreDx.rgb()) != 1:
                mDc(velStuck, velStuck)
        else:
            angPSx = motoreSx.angle()
            angPDx = motoreDx.angle()

def DEBUG():
    while True:
        dist = sensoreDist.distance()
        isTouch = False
        rgbSx = sensoreSx.rgb()
        rgbDx = sensoreDx.rgb()
        colSx = assCol(rgbSx)
        colDx = assCol(rgbDx)
        isG = 0
        if dist <= 55:
            isTouch = True
        if colSx == 2 or colDx == 2:
            isG = 1
        debug = "isGreen : " + str(isG) + "\n" + "Sx : " + str(rgbSx) + "\n" + "Dx : " + str(rgbDx) + "\n" + str(colSx) + " ! " + str(colDx) + "\n\n\n\n"
        debug = debug + "\nvalWNero : " + str(valWNero) + "\nDist : " + str(dist) + " | isTouch : " + str(isTouch)
        print(debug)
def ostacolo():
    tempOst = 1
    while tempOst:
        mDc(velMin, velMin)
        if sensoreDist.distance() < 110:
            tempOst = 0
    mAng(pManov, ang1CM * -5, ang1CM * -5, Stop.HOLD, True)
    mAng(pManov, ang180 / 3, -ang180 / 3, Stop.HOLD, True)
    mDc(velOstB, velOstF)
    tempOst = 1
    while tempOst:
        if assCol(sensoreDx.rgb()) == 0:
            tempOst = 0
            mAng(pManov, ang1CM * 2, ang1CM * 2, Stop.HOLD, True)
            while assCol(sensoreSx.rgb()) != 0:
                mDc(50, -50)


        
while True:
    bDx = bDx
    bSx = bSx
    dist = sensoreDist.distance()
    isTouch = False
    rgbSx = sensoreSx.rgb()
    rgbDx = sensoreDx.rgb()
    colSx = assCol(rgbSx)
    colDx = assCol(rgbDx)
    sMSx = motoreSx.speed()
    sMDx = motoreDx.speed()
    isG = 0
    if dist <= 55:
        isTouch = True
    valWNCheck = 0
    valWVCheck = 0
    if (valWNero  + tempoW) <= Watch.time():
        valWNCheck = 1
    debug = "isGreen : " + str(isG) + "\n" + "Sx : " + str(rgbSx) + "\n" + "Dx : " + str(rgbDx) + "\n" + str(colSx) + " ! " + str(colDx) + "\n\n\n\n"
    debug = debug + "\nvalWNCheck : " + str(valWNCheck) + " | valWNero : " + str(valWNero) + " | valWNeroS : " + str((valWNero + tempoW)) + "\nDist : " + str(dist)
    debug = debug + "\nmSpeed : " + str(sMSx) + " | " + str(sMDx)
    print(debug)

    # Inizio azioni
    # Verde Sx
    if (colSx == 2 and valWNCheck):
        if verGreen(sensoreSx):
            ev3.speaker.beep(200, 50)
            SLineSx()
            ev3.speaker.beep(100, 50)
    # Verde Dx
    elif (colDx == 2 and valWNCheck):
        if verGreen(sensoreDx):
            ev3.speaker.beep(200, 50)
            SLineDx()
            ev3.speaker.beep(100, 50)
    # Doppio Nero
    elif (colSx == 0 and colDx == 0):
        valWNero = Watch.time()
    # Doppio Verde Rimosso
    # Nero Sx
    elif (colSx == 0 and colDx != 0):
        bSx = bSx + 0
        bDx = 0
        mSx()
    # Nero Dx
    elif (colDx == 0 and colSx != 0  ):
        bDx = bDx + 0
        bSx = 0
        mDx()
    # Eccezione
    else:
        mDc(velF, velF)

    if (sensoreDist.distance() < 130):
        ostacolo()
    
    stuckFix()