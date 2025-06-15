import sys

import time

from VehicleMav import Submarine

import threading

from queue import Queue

# from CircleDetector import BlueCircleDetector

import csv

import signal

import datetime







class PIDController:

    def __init__(self, kp, ki, kd, windup_guard=None, ramp_rate=None, filter_coefficient=None):

        self.Kp = kp

        self.Ki = ki

        self.Kd = kd

        self.windup_guard = windup_guard

        self.ramp_rate = ramp_rate

        self.filter_coefficient = filter_coefficient

        self.last_error = None

        self.integral = 0

        self.last_output = 0



    def update(self, error, dt):

        if self.last_error is not None:

            derivative = (error - self.last_error) / dt

        else:

            derivative = 0

        # Anti-windup mechanism

        self.integral += error * dt

        if self.windup_guard is not None:

            self.integral = max(min(self.integral, self.windup_guard), -self.windup_guard)

        # Filter coefficient for integral term

        if self.filter_coefficient is not None:

            self.integral = self.filter_coefficient * self.integral + (1 - self.filter_coefficient) * error * dt

        # PID output calculation

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        # Ramp rate limit



        output = output

        self.last_output = output

        self.last_error = error

        return output



    def tune(self, method, *args):

        if method == "Ziegler-Nichols":

            self._kp = 0.6 * self.Kp

            self._ki = 1.2 * self.Kp / self.Ki

            self._kd = 0.075 * self.Kp * self.Kd

            self.tuned = True

            return self._kp, self._ki, self._kd





class Autonomous:

    def __init__(self, rov, pid_pitch, pid_roll, pid_altitude, pid_yaw): #, pid_throttle

        self.vehicle = rov

        self.pid_pitch = pid_pitch

        self.pid_roll = pid_roll

        self.pid_altitude = pid_altitude

        self.pid_yaw = pid_yaw

       # self.pid_throttle= pid_throttle

        self.targetPitch = 0

        self.targetRoll = 0

        self.targetYaw = 0

        self.targetThrottle = 1500  # Neutral position

        self.targetPressure = 1040 #convert from pascal to cm/m

        self.target_alt = 300





        self.is_pitch_stable = False

        self.is_roll_stable = False

        self.is_altitude_stable = False

        self.is_yaw_stable = False



    def initialization(self):

        time.sleep(1)

        self.vehicle.Channel1 = 1500

        self.vehicle.Channel2 = 1500

        self.vehicle.Channel3 = 1500

        self.vehicle.Channel4 = 1500

        self.vehicle.Channel5 = 1500

        self.vehicle.Channel6 = 1500

        time.sleep(1)



    def stablePitch(self):

        startTime = time.time()

        old_time = time.time()

        current_time = time.time()



        while True:

            time.sleep(0.1)

            current_time = time.time()

            dt = current_time - old_time

            error = self.targetPitch - self.vehicle.rotation.pitch

            output = self.pid_pitch.update(error, dt)

            self.vehicle.Channel4 = 1500 - min(400, max(-400, output))

            self.is_pitch_stable = abs(error) < 3

            print(f"Pitch -> Error: {error}, Output: {output}, Pitch: {self.vehicle.rotation.pitch}")

            old_time = current_time



    def stableRoll(self):

        startTime = time.time()

        old_time = time.time()

        current_time = time.time()



        while True:

            time.sleep(0.1)

            current_time = time.time()

            dt = current_time - old_time

            error = self.targetRoll - self.vehicle.rotation.roll

            output = self.pid_roll.update(error, dt)

            self.vehicle.Channel2 = 1500 + min(400, max(-400, output))

            self.is_roll_stable = abs(error) < 3

            print(f"Roll -> Error: {error}, Output: {output}, Roll: {self.vehicle.rotation.roll}")

            old_time = current_time



    def stableAltitude(self):

        startTime = time.time()

        old_time = time.time()

        current_time = time.time()



        while True:

            time.sleep(0.1)

            current_time = time.time()

            dt = current_time - old_time

            error = self.vehicle.scalePressure.press_abs - self.targetPressure

            #error = self.vehicle.globalPosition.relative_alt - self.target_alt

            output = self.pid_altitude.update(error, dt)

            self.vehicle.Channel3 = 1500 + min(400, max(-400, output))

            self.is_altitude_stable = abs(error) < 3

            print(f"Altitude -> Error: {error}, Output: {output}, alt: {self.vehicle.globalPosition.relative_alt}")

            old_time = current_time



    def stableYaw(self, target_yaw=None):

        startTime = time.time()

        if target_yaw is not None:

            self.targetYaw = target_yaw



        #         if target_yaw + self.targetYaw > 360:

    #             self.targetYaw = (target_yaw + self.targetYaw) - 360

    #         else:



        old_time = time.time()

        startTime = time.time()

        time.sleep(2)

        current_time = time.time()



        while True:

            time.sleep(0.1)

            current_time = time.time()

            dt = current_time - old_time



            # Calculate yaw error and handle angle wrapping

            error = self.targetYaw - self.vehicle.rotation.yaw

            if error > 180:

                error -= 360

            elif error < -180:

                error += 360



            # Update PID output

            output = self.pid_yaw.update(error, dt)

            self.vehicle.Channel4 = 1500 + min(400, max(-400, output))  # Adjust yaw control channel



            # Check if yaw is stable within a 3-degree threshold

            self.is_yaw_stable = abs(error) < 3

            if self.is_yaw_stable:

                break  # Stop once yaw is stable



            print(

                f"Yaw -> Target: {self.targetYaw}, Error: {error}, Output: {output}, Yaw: {self.vehicle.rotation.yaw}")

            old_time = current_time



    # def stableThrottle(self):

    #     old_time = time.time()

    #     while True:

    #         time.sleep(0.1)

    #         current_time = time.time()

    #         dt = current_time - old_time

    #         error = self.targetThrottle - self.vehicle.Channel5

    #         output = self.pid_throttle.update(error, dt)

    #         self.vehicle.Channel5 = min(1900, max(1100, 1500 + output))

    #         # print(f"Throttle -> Error: {error}, Output: {output}, Throttle: {self.vehicle.Channel3}")

    #         old_time = current_time



    def stabilize(self):

        pitch_thread = threading.Thread(target=self.stablePitch, args=())

        roll_thread = threading.Thread(target=self.stableRoll, args=())

        altitude_thread = threading.Thread(target=self.stableAltitude, args=())

        yaw_thread = threading.Thread(target=self.stableYaw, args=())

        # throttle_thread = threading.Thread(target=self.stableThrottle, args=())





        pitch_thread.start()

        roll_thread.start()

        altitude_thread.start()

        yaw_thread.start()

        # throttle_thread.start()





        pitch_thread.join()

        roll_thread.join()

        altitude_thread.join()

        yaw_thread.join()

        # throttle_thread.join()



        self.STOP()



    def moveForward(self, intended_time, signal_multiplier):

        self.vehicle.Channel5 = 1500 + signal_multiplier

        time.sleep(intended_time)

        self.vehicle.Channel5 = 1500

        time.sleep(0.1)



    def moveBackward(self, intended_time, signal_multiplier):

        self.vehicle.Channel5 = 1500 - signal_multiplier

        time.sleep(intended_time)

        self.vehicle.Channel5 = 1500

        time.sleep(0.1)



    def goForward(self, intended_time, speed_multiplier):

        start_time = time.time()

        print("Motors are starting...")

        self.vehicle.Channel5 = max(1500, min(2000, 1500 + speed_multiplier))

        while time.time() - start_time < intended_time:

            time.sleep(0.01)

            print("accx: ", self.vehicle.scaleImu.xacc, "accy: ", self.vehicle.scaleImu.yacc, "accz: ",

                  self.vehicle.scaleImu.zacc)

            if time.time() - start_time > 0.5 and abs(

                    self.vehicle.scaleImu.xacc) < 0.5 and time.time() - start_time < 1:

                print("Vehicle is stuck")

                return False

        self.vehicle.Channel5 = 1500 - speed_multiplier

        time.sleep(0.5)

        self.vehicle.Channel5 = 1500

        self.vehicle.Channel3 = 1500

        time.sleep(0.1)

        return True



    def goBackward(self, intended_time, speed_multiplier):

        start_time = time.time()

        self.vehicle.Channel5 = max(1500, min(2000, 1500 - speed_multiplier))

        self.vehicle.Channel3 = 1500

        while time.time() - start_time < intended_time:

            time.sleep(0.01)

            print("accx: ", self.vehicle.scaleImu.xacc, "accy: ", self.vehicle.scaleImu.yacc, "accz: ",

                  self.vehicle.scaleImu.zacc)

            if time.time() - start_time > 0.5 and abs(

                    self.vehicle.scaleImu.xacc) < 0.5 and time.time() - start_time < 1:

                print("Vehicle is stuck")

                return False

        self.vehicle.Channel3 = 1500 + speed_multiplier

        time.sleep(0.5)

        self.vehicle.Channel5 = 1500

        self.vehicle.Channel3 = 1500

        time.sleep(0.1)

        return True



    def throttle(self, intended_time, signal_multiplier):

        self.vehicle.Channel2 = 1500



        if signal_multiplier > 1100 and signal_multiplier < 1900:

            signal_multiplier = signal_multiplier - 1500

        self.vehicle.Channel4 = 1500 + signal_multiplier



        time.sleep(intended_time)

        self.vehicle.Channel2 = 1500

        self.vehicle.Channel4 = 1500

        time.sleep(0.1)



    def stayIdle(self):

        self.vehicle.Channel1 = 1500

        self.vehicle.Channel2 = 1500

        self.vehicle.Channel3 = 1500

        self.vehicle.Channel4 = 1500

        self.vehicle.Channel5 = 1500

        self.vehicle.Channel6 = 1500



    def STOP(self):

        self.vehicle.Channel1 = 1500

        self.vehicle.Channel2 = 1500

        self.vehicle.Channel3 = 1500

        self.vehicle.Channel4 = 1500

        self.vehicle.Channel5 = 1500

        self.vehicle.Channel6 = 1500



    def goForwardWithInterrupt(self, signal_multiplier, going_forward_queue, still_going_forward_queue):

        self.vehicle.Channel5 = max(1000, min(2000, 1500 + 100 * signal_multiplier))

        while True:

            time.sleep(0.01)



            if not going_forward_queue.empty():

                can_go_forward = going_forward_queue.get()



                if not can_go_forward:

                    self.vehicle.Channel3 = 1500 - 100 * signal_multiplier

                    self.vehicle.Channel5 = 1500 - 100 * signal_multiplier

                    time.sleep(0.5)

                    self.vehicle.Channel3 = 1500

                    time.sleep(0.1)

                    still_going_forward_queue.put(False)

                else:

                    self.vehicle.Channel3 = 1500 - 100 * signal_multiplier

                    self.vehicle.Channel5 = 1500 + 100 * signal_multiplier

                    time.sleep(0.5)

                    self.vehicle.Channel3 = 1500

                    time.sleep(0.1)



                    still_going_forward_queue.put(True)



    def pitch(self, value):

        self.vehicle.channel6 = value

        time.sleep(12)



    def roll(self, value):

        self.vehicle.channel5 = value

        time.sleep(12)



    # pitch does not work here but yaw does. why!?

    def yaw(self, intended_time, signal_multiplier):

        self.vehicle.Channel6 = 1500

        self.vehicle.Channel2 = 1500 + signal_multiplier

        time.sleep(intended_time)

        self.vehicle.Channel3 = 1500

        self.vehicle.Channel6 = 1500

        time.sleep(0.1)





def stopVehicle(auto, seconds):

    auto.stayIdle()

    time.sleep(seconds)





def printPressure(auto, filename="pressure_data.csv"):

    global stop_threads

    # Open the file in write mode to clear previous contents and write the header

    with open(filename, mode='w', newline='') as file:

        writer = csv.writer(file)

        writer.writerow(["Pressure (mbar)", "current time", "x", "y", "z", "channel_4"])  # Write header once



    # Reopen the file in append mode for continuous logging

    with open(filename, mode='a', newline='') as file:

        writer = csv.writer(file)

        while not stop_threads:

            pressure = auto.vehicle.scalePressure.press_abs

            z = auto.vehicle.localPosition.z

            x = auto.vehicle.localPosition.x

            y = auto.vehicle.localPosition.y

            current_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

            writer.writerow([pressure, current_time, x, y, z, auto.vehicle.Channel4])

            file.flush()  # Ensure data is written to the file

            print("\n\n\n\n\n\n\n\\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n"*5)

            time.sleep(1)





def signal_handler(sig, frame):

    global stop_threads

    stop_threads = True

    print("Exiting gracefully...")

    sys.exit(0)





def main():

    global stop_threads

    stop_threads = False



    signal.signal(signal.SIGINT, signal_handler)



    rov = Submarine('/dev/ttyACM0')

    rov.setVehicleModeTo("stabilize")



    time.sleep(0.5)

    print("Vehicle is ready!")



    time.sleep(1)



    rov.arm()

    print("Vehicle Armed!")



    pid = PIDController(1, 0.01, 0.1, windup_guard=20, ramp_rate=10, filter_coefficient=0.5)

    auto = Autonomous(rov, pid, pid, pid, pid)  #, pid

    auto.initialization()

    print("Initialization complete!")



    t2 = threading.Thread(target=printPressure, args=(auto,))

    t2.start()

    print("\n\nt2 started....")

    print("\n\nTesting after 3 seconds......")

    time.sleep(30)



    print("going down for 20 seconds...")

    # auto.throttle(60, 1352)

    # auto.STOP()

    # auto.throttle(5, 1700)

    print("--main-- Test is COMPELETED!")





def testStability():

    rov = Submarine('/dev/ttyACM0')

    rov.setVehicleModeTo("manual")



    time.sleep(0.5)

    print("Vehicle is ready!")

    time.sleep(2)



    rov.arm()

    print("Vehicle Armed!")



    pid_pitch = PIDController(kp=1.5, ki=0.4, kd=0.05, windup_guard=20, ramp_rate=5, filter_coefficient=0.5) #1.5

    pid_roll = PIDController(kp=3, ki=0.4, kd=0.03, windup_guard=15, ramp_rate=5, filter_coefficient=0.5)

    pid_altitude = PIDController(kp=2, ki=0.4, kd=0.05, windup_guard=30, ramp_rate=5, filter_coefficient=0.5)

    pid_yaw = PIDController(kp=10, ki=0.4, kd=0.1, windup_guard=15, ramp_rate=5, filter_coefficient=0.5)

    # pid_throttle = PIDController(kp=2, ki=0.3, kd=0.1, windup_guard=20, ramp_rate=5, filter_coefficient=0.5)





    auto = Autonomous(rov, pid_pitch, pid_roll, pid_altitude, pid_yaw) #, pid_throttle

    auto.initialization()

    print("Initialization complete!")



    print("Testing stabilize() with dynamic throttle control")

    start = time.time()



    try:

        # Start stabilization in a separate thread

        #stabilize_thread = threading.Thread(target=auto.stabilize)

        #stabilize_thread.start()



        #auto.vehicle.channel4 = 1900

        # auto.stableYaw(target_yaw=90)

        # print("\n\YAW\n\n")

        # auto.STOP()

        # print("\n\nSTOP\n\n")

        # time.sleep(10)

    

        auto.stablePitch()

        # print("\n\PITCH\n\n")

        # auto.STOP()

        # print("\n\nSTOP\n\n")

        # time.sleep(10)

        

        # auto.stableRoll

        # print("\n\ROLL\n\n")

        # auto.STOP()

        # print("\n\nSTOP\n\n")

        # time.sleep(10)

        

        # auto.stableAltitude()

        # print("\n\nALTITUDE\n\n")

        # auto.STOP()

        # print("\n\nSTOP\n\n")

        # time.sleep(10)

        

        # auto.throttle(20, 400)

		

        # auto.moveBackward(20, 200)

        # print(f"Updating target yaw to 90 degrees.")

        # # Turn 90 degrees to the right

        # auto.stableYaw(target_yaw=90)



        # auto.moveForward(15, 200)

        # auto.moveBackward(2, 200)

        # print(f"Updating target yaw to 180 degrees.")

        # # Turn 180 degrees

        # auto.stableYaw(target_yaw=90)



        # auto.moveForward(15, 200)

        # auto.moveBackward(2, 200)

        # print(f"Updating target yaw to -90 degrees.")

        # # Turn 180 degrees

        # auto.stableYaw(target_yaw=180)

    



    except KeyboardInterrupt:

        auto.vehicle.disarm()

        print("Vehicle Disarmed!")

        print("Stabilization interrupted by user.")

    finally:

        auto.STOP()

        print("Vehicle Stopped!")



    end = time.time()

    print(f"Stabilization test completed in {end - start:.2f} seconds.")



def testyeniarac():

    channels = ["Channel1", "Channel2", "Channel3", "Channel4", "Channel5", "Channel6"]

    rov = Submarine('/dev/ttyACM0')

    rov.setVehicleModeTo("manual")



    time.sleep(0.5)

    print("Vehicle is ready!")

    time.sleep(2)



    rov.arm()

    print("Vehicle Armed!")

    

    pid_pitch = PIDController(kp=1.5, ki=0.4, kd=0.05, windup_guard=20, ramp_rate=5, filter_coefficient=0.5) #1.5

    pid_roll = PIDController(kp=3, ki=0.4, kd=0.03, windup_guard=15, ramp_rate=5, filter_coefficient=0.5)

    pid_altitude = PIDController(kp=2, ki=0.4, kd=0.05, windup_guard=30, ramp_rate=5, filter_coefficient=0.5)

    pid_yaw = PIDController(kp=10, ki=0.4, kd=0.1, windup_guard=15, ramp_rate=5, filter_coefficient=0.5)



    auto = Autonomous(rov, pid_pitch, pid_roll, pid_altitude, pid_yaw) 

    auto.initialization()

    print("Initialization complete!")



    print("Testing stabilize() with dynamic throttle control")

    start = time.time()



    try:

        for channel in channels:

            rov.vehicle.channel = 1900

            time.sleep(5)

            rov.vehicle.channel = 1100

            

    except KeyboardInterrupt:

        print("test interrupted by user.")

    finally:

        auto.STOP()

        auto.vehicle.disarm()

        print("Vehicle Disarmed!")

        print("Vehicle Stopped!")

    

    end = time.time()



def testTargetDetection():

    rov = Submarine('/dev/ttyACM0')

    rov.setVehicleModeTo("manual")



    time.sleep(0.5)

    print("Vehicle is ready!")

    time.sleep(70)



    rov.arm()

    print("Vehicle Armed!")



    pid_pitch = PIDController(kp=1.5, ki=0.4, kd=0.05, windup_guard=20, ramp_rate=5, filter_coefficient=0.5)

    pid_roll = PIDController(kp=3, ki=0.4, kd=0.03, windup_guard=15, ramp_rate=5, filter_coefficient=0.5)

    pid_altitude = PIDController(kp=6, ki=0.4, kd=0.1, windup_guard=30, ramp_rate=5, filter_coefficient=0.5)

    pid_yaw = PIDController(kp=10, ki=0.4, kd=0.1, windup_guard=15, ramp_rate=5, filter_coefficient=0.5)



    objectQueue = Queue()

    targetObjectQueue = Queue()



    auto = Autonomous(rov, pid_pitch, pid_roll, pid_altitude, pid_yaw)

    auto.initialization()

    print("Initialization complete!")



    print("Testing target detection and approach")

    start = time.time()



    try:

        stabilize_thread = threading.Thread(target=auto.stabilize)

        stabilize_thread.start()



        circleDetector = BlueCircleDetector(0, calibrationFactor=0.017361)

        circle_thread = threading.Thread(target=circleDetector.run)

        circle_thread.start()



        searching = True



        while searching:

            print("Searching for the target...")

            time.sleep(30)

            try:

                while searching:  # Perform full 360-degree rotation in 30-degree steps

                    new_yaw = auto.vehicle.rotation.yaw

                    new_yaw += 30

                    auto.stableYaw(target_yaw=new_yaw)

                    time.sleep(0.1)

                    objectData = circleDetector.get_target_data()

                    horz_dist = (objectData.horz_dist/10)

                    updated_horz_dist = auto.vehicle.rotation.yaw + horz_dist

                    print(f"Approaching target at angle: {horz_dist}")

                    print(f"New Angle: {updated_horz_dist}")



                    while objectData and abs(objectData.distance) > 0.1:

                        print("Object detected!")

                        print("Target Circle found!")



                        while abs(horz_dist) > 3:

                            print(f"Turning: {horz_dist} degrees")

                            auto.stableYaw(target_yaw=((updated_horz_dist)))

                            time.sleep(0.1)

                            objectData = circleDetector.get_target_data()

                            horz_dist = (objectData.horz_dist / 10)

                            updated_horz_dist = auto.vehicle.rotation.yaw + horz_dist



                        auto.moveForward(2, 200)



                        while abs(objectData.distance) > 0.1:

                            objectData = circleDetector.get_target_data()

                            horz_dist = (objectData.horz_dist / 10)

                            if horz_dist == 0:

                                print(f"horz distance{horz_dist}")

                                x = 0

                                x += 1

                                if x == 100:

                                    break

                            else:

                                x = 0

                            updated_horz_dist = auto.vehicle.rotation.yaw + horz_dist



                            # If the horizontal distance is large, adjust the yaw

                            while abs(horz_dist) > 5:

                                print(f"Correcting yaw while moving: {horz_dist} degrees")

                                auto.stableYaw(target_yaw=updated_horz_dist)

                                time.sleep(0.1)



                            if objectData.distance > 0.7:

                                print("Moving forward cuz 0.7")

                                auto.moveForward(1, 200)

                            elif objectData.distance < 1.2:

                                auto.moveBackward(1, 200)

                            else:

                                while abs(horz_dist) != 0:

                                    print(f"Turning: {horz_dist} degrees")

                                    auto.stableYaw(target_yaw=((updated_horz_dist)))

                                    time.sleep(0.1)

                                    objectData = circleDetector.get_target_data()

                                    horz_dist = (objectData.horz_dist / 10)

                                    updated_horz_dist = auto.vehicle.rotation.yaw + horz_dist

                                print("We are at the end")

                                break

                            time.sleep(0.1)



                    else:

                        print("False detection. Continuing search...")

                else:

                    print("No object detected. Continuing search...")



                if not searching:

                    break

            except Exception as e:

                print(f"Error: {e}")

                break





            auto.stayIdle()

            print("Target reached!")



        stabilize_thread.join()

        circle_thread.join()



    except KeyboardInterrupt:

        print("Stabilization interrupted by user.")

    finally:

        auto.STOP()

        print("Vehicle Stopped!")



    end = time.time()

    print(f"Target detection test completed in {end - start:.2f} seconds.")



if __name__ == "__main__":

    #testTargetDetection()

    #testStability()

    testyeniarac()

    #    testGorev1()



    # main()



    # rov = Submarine('/dev/ttyACM0')

    # rov.setVehicleModeTo("manual")



    # time.sleep(0.5)

    # print("Vehicle is ready!")



    # rov.arm()

    # print("Vehicle Armed!")



    # pid = PIDController(1, 0.01, 0.1, windup_guard=20, ramp_rate=10, filter_coefficient=0.5)

    # auto = Autonomous(rov, pid)

    # auto.initialization()

    # print("Initialization complete!")



    # print("Testing after 30 seconds......")

    # time.sleep(0)

    # start = time.time()

    # while time.time() - start < 60:

    #     auto.stabilize()


