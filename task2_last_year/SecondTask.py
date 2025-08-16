import time
from AbisTest import PIDController, Autonomous, stopVehicle, printPressure, signal_handler
from task2_last_year.line_shape_detector import LineShapeDetector
from queue import Queue
from threading import Thread, Event, Lock
# from testfunctions import stabilize, autonomous, stayIdle, STOP, stableYaw, moveForward, moveBackward
from Vehicle import Submarine


rov = Submarine('/dev/ttyACM0')
rov.setVehicleModeTo("manual")

time.sleep(0.5)
print("Starting test")
time.sleep(45)

rov.arm()
print("Vehicle is ready")
time.sleep(1)

print("vehicle armed")

pid_pitch = PIDController(kp=1.5, ki=0.4, kd=0.05, windup_guard=20, ramp_rate=5, filter_coefficient=0.5)
pid_roll = PIDController(kp=3, ki=0.4, kd=0.03, windup_guard=15, ramp_rate=5, filter_coefficient=0.5)
pid_altitude = PIDController(kp=6, ki=0.4, kd=0.1, windup_guard=30, ramp_rate=5, filter_coefficient=0.5)
pid_yaw = PIDController(kp=10, ki=0.4, kd=0.1, windup_guard=15, ramp_rate=5, filter_coefficient=0.5)

auto = Autonomous(rov, pid_pitch, pid_roll, pid_altitude, pid_yaw)
auto.initialization()
print("Initialization complete!")

print("Starting autonomous flight")
time.sleep(1)

start = time.time()

goForwardQueue = Queue()
stillGoingForwardQueue = Queue()

stabilize_thread = Thread(target=auto.stabilize, daemon=True)
autonomous_thread = Thread(target=auto.goForwardWithInterrupt, daemon=True)
LineShape_detector = LineShapeDetector(0)
detector_thread = Thread(target=LineShape_detector.run, daemon=True)

stabilize_thread.start()
autonomous_thread.start()

detector_thread.start()
try:
    folow_line = False
    searching = True
    targetYaw = 0
    while searching:
        print("Searching for the target...")
        time.sleep(5)
        auto.stableYaw(target_yaw=0)
        time.sleep(0.1)

        auto.vehicle.Channel5 = 1600 # go forward slowly to find a line
        print("auto.vehicle.Channel5 = 1600") # go forward slowly to find a line
        line_slices = LineShape_detector.slice_images
        
        front_slices = line_slices[:5] # Total is lineShape_detector.num_slices == 15
        # 0-4 front, 5-9 middle, 10-14 back

        slice_counter = 0 # counts how many of first slices 
        for slice in front_slices: # check first 5 slices. 
            if slice.includes_line:
                slice_counter += 1
        
        if slice_counter >= 3: # if more than 3 slices include line (black contour)
            folow_line = True

        while folow_line:
            startOfLine = -1
            for slice in front_slices: 
                startOfLine += 1
                if slice.includes_line:
                    break # find the first part of the line
                else:
                    startOfLine = -1

            if startOfLine != -1: # if there is no line in front
                head_of_line_0 = line_slices[startOfLine].error # is 100 if at right most of the frame
                head_of_line_1 = line_slices[startOfLine+1].error
                head_of_line_3 = line_slices[startOfLine+2].error
            else:
                head_of_line_0 = 0

            middle = int(LineShape_detector.num_slices/2) - 1 # should be 6 for 15
            error_middle_1 = line_slices[middle].error - line_slices[middle+2].error 
            error_middle_2 = line_slices[middle+1].error - line_slices[middle+3].error
            error_middle_3 = line_slices[middle+2].error - line_slices[middle+4].error

            
            if (error_middle_1>30 and error_middle_2>30) or (error_middle_2>30 and error_middle_3>30): 
                new_angle = targetYaw+90
                # stableYaw(target_yaw=new_angle)
                print("turn right")
            elif error_middle_1<-30 and error_middle_2<-30 or error_middle_2<-30 and error_middle_3<-30:
                new_angle = targetYaw-90
                # stableYaw(target_yaw=new_angle)          
                print("turn left")
            # maybe should create a thread for this: 
            elif abs(head_of_line_0)>15 and abs(head_of_line_3)>15: # trying to follow line by correcting the yaw
                error_angel = head_of_line_0/4
                targetYaw = error_angel
                print("targstabetYaw: ", targetYaw)


    stabilize_thread.join()
    detector_thread.join()
    # STOP()
    end = time.time()
    print(f"Target detection test completed in {end - start:.2f} seconds.")

except KeyboardInterrupt:
    print("Stabilization interrupted by user.")

finally:
    stabilize_thread.join()
    autonomous_thread.join()
    detector_thread.join()
    auto.STOP()
    print("Vehicle Stopped!")


