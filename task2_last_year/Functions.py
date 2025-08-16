import serial
from pymavlink import mavutil
import time
from queue import Queue
import threading
import numpy as np
from task2_last_year.SlicedImage import *


def findSmallestCircle(objectQueue, cameraQueue):
    # compare the circles and find the smallest one
    while True:
        if not objectQueue.empty():
            objects = objectQueue.get()
            if objects:
                smallestCircle = None
                for obj in objects:
                    if obj.shape == "circle":
                        if smallestCircle is None:
                            smallestCircle = obj
                        elif obj.radius < smallestCircle.radius:
                            smallestCircle = obj
                if smallestCircle:
                    cameraQueue.put(smallestCircle)
                    break


def launchFiveTorpedoes(launchingComplexQueue):
    for i in range(5):
        activateLaunchingComplex(launchingComplexQueue)
        print("Launching torpedo", i + 1)
        time.sleep(1)


def activateLaunchingComplex(lauchingComplexQueue):
    readFromComPort(lauchingComplexQueue)
    if lauchingComplexQueue.get():
        print("Launching complex is activated!")
    else:
        print("Failed to activate launching complex!")


def readFromComPort(launchingComplexQueue):
    pass


def moveVehicle(movement, angle_queue, altitude_queue, pressure_queue):
    thread_stable_tilt = threading.Thread(target=movement.stableTilt)
    thread_backward = threading.Thread(target=movement.goBackward, args=(2, 1))
    thread_forward = threading.Thread(target=movement.goForward, args=(2, 1))
    thread_left = threading.Thread(target=movement.goLeft, args=(2, 1))
    thread_right = threading.Thread(target=movement.goRight, args=(2, 1))
    thread_down = threading.Thread(target=movement.goDown, args=(2, 1))
    thread_up = threading.Thread(target=movement.goUp, args=(2, 1))
    thread_rotate = threading.Thread(target=movement.rotate, args=(90, 1))
    thread_stable_angle = threading.Thread(target=movement.stableAngle, args=(angle_queue,))
    thread_stable_altitude = threading.Thread(target=movement.stableAltitude, args=(altitude_queue, pressure_queue))

    print("Starting threads...")
    thread_stable_tilt.start()
    thread_backward.start()
    thread_forward.start()
    thread_left.start()
    thread_right.start()
    thread_down.start()
    thread_up.start()
    thread_rotate.start()
    thread_stable_angle.start()
    thread_stable_altitude.start()

    thread_stable_tilt.join()
    thread_backward.join()
    thread_forward.join()
    thread_left.join()
    thread_right.join()
    thread_down.join()
    thread_up.join()
    thread_rotate.join()
    thread_stable_angle.join()
    thread_stable_altitude.join()

    print("All threads are done!")


def SlicePart(im, images, slices):
    height, width = im.shape[:2]
    sl = int(height / slices)

    for i in range(slices):
        part = sl * i
        crop_img = im[part:part + sl, 0:width]
        images[i].image = crop_img
        images[i].Process()


def RepackImages(images):
    img = images[0].image
    for i in range(len(images)):
        if i == 0:
            img = np.concatenate((img, images[1].image), axis=0)
        if i > 1:
            img = np.concatenate((img, images[i].image), axis=0)

    return img


def Center(moments):
    if moments["m00"] == 0:
        return 0

    x = int(moments["m10"] / moments["m00"])
    y = int(moments["m01"] / moments["m00"])

    return x, y

def trackLine():
    pass

def findAnomalies():
    pass

#------ line detedciont functions ------#

def SlicePart(im, images, slices):
    height, width = im.shape[:2]
    sl = int(height/slices)
    
    for i in range(slices):
        part = sl*i
        crop_img = im[part:part+sl, 0:width]
        images[i].image = crop_img
        images[i].Process()
    
def RepackImages(images):
    img = images[0].image
    for i in range(len(images)):
        if i == 0:
            img = np.concatenate((img, images[1].image), axis=0)
        if i > 1:
            img = np.concatenate((img, images[i].image), axis=0)
            
    return img

def Center(moments):
    if moments["m00"] == 0:
        return 0
        
    x = int(moments["m10"]/moments["m00"])
    y = int(moments["m01"]/moments["m00"])

    return x, y
