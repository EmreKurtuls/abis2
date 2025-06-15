import random
import datetime
from dronekit import connect, VehicleMode
import math
import time
from pymavlink import mavutil


class Submarine:
    vehicle = None
    somenumberforme = 0
    sim_mode = False

    class rawimu:
        xacc = 0
        yacc = 0
        zacc = 0
        xgyro = 0
        ygyro = 0
        zgyro = 0
        xmag = 0
        ymag = 0
        zmag = 0

    class rotation:
        pitch = 0
        yaw = 0
        roll = 0

    class localPosition:
        x = 0
        y = 0
        z = 0
        vx = 0
        vy = 0
        vz = 0
        
    class globalPosition:
        time_boot_ms = 0
        lat = 0
        lon = 0
        alt = 0
        relative_alt = 0
        vx = 0
        vy = 0
        vz = 0
        

    class scalePressure:
        press_abs = 0
        press_diff = 0
        temperature = 0

    class scaleImu:
        sometime = 0
        lasttime = 0
        timediff = 0
        xacc = 0
        yacc = 0
        zacc = 0
        xgyro = 0
        ygyro = 0
        zgyro = 0

    class test_speed:
        angle_from_gnd = 0
        x = 0
        y = 0
        z = 0
        velx = 0

    def __init__(self, Usb="COM10", isSim=False, ip="127.0.0.1", port=5762):

        self.sim_mode = isSim
        try:
            if self.sim_mode:
                self.vehicle = connect("tcp:" + ip + ":" + str(port), wait_ready=False)
            else:
                self.vehicle = connect("/dev/ttyACM1", wait_ready=True, baud=115200)

        except Exception as e:
            print("Error: ", e)
            self.vehicle = None
            return

        self.vehicle.add_message_listener("ATTITUDE", self.attitudeListener)

        self.vehicle.add_message_listener("RAW_IMU", self.rawImuListener)

        self.vehicle.add_message_listener(
            "LOCAL_POSITION_NED", self.localPositionListener
        )
            
        self.vehicle.add_message_listener(
            "GLOBAL_POSITION_INT", self.globalPositionListener
        )

        self.vehicle.add_message_listener(
            "SCALED_PRESSURE2", self.scalePressureListener
        )

        self.vehicle.add_message_listener("SCALED_IMU2", self.scaledImu2Listener)

    def setVehicleModeTo(self, modeName: str, Force=False):
        if self.vehicle is not None:
            while True:
                if self.vehicle.mode.name != modeName or Force:
                    try:
                        self.vehicle.mode = VehicleMode(modeName.upper())
                        print(f"Changed mode to {modeName}")
                        Force = True
                        break

                    except Exception as e:
                        print(f"Error setting mode: {e}")
                        break
                else:
                    print(f"Already in {modeName} mode")
                    break
        else:

            print("Not Connected")

    def getVehicleMode(self):

        try:

            if self.vehicle is not None:

                print("Vehicle Mode: ", self.vehicle.mode.name)

                return str(self.vehicle.mode.name)

            else:

                return "Not Connected"

        except Exception as e:

            print("Error: ", e)

            return "Error"

    def scaledImu2Listener(self, msgType, vehicleAdd, msg):

        self.scaleImu.sometime = msg.time_boot_ms / 100.0

        self.scaleImu.timediff = self.scaleImu.sometime - self.scaleImu.lasttime

        self.scaleImu.xacc = float("{:.4f}".format(msg.xacc)) / 1000.0

        self.scaleImu.yacc = float("{:.4f}".format(msg.yacc)) / 1000.0

        self.scaleImu.zacc = float("{:.4f}".format(msg.zacc)) / 1000.0

        self.scaleImu.xgyro = float("{:.4f}".format(msg.xgyro)) / 1000.0

        self.scaleImu.ygyro = float("{:.4f}".format(msg.ygyro)) / 1000.0

        self.scaleImu.zgyro = float("{:.4f}".format(msg.zgyro)) / 1000.0

        self.test_speed.angle_from_gnd = math.atan2(
            self.scaleImu.zacc,
            math.sqrt((self.scaleImu.yacc**2) + (self.scaleImu.xacc**2)),
        )

        self.test_speed.angle_from_gnd = abs(
            abs(math.degrees(self.test_speed.angle_from_gnd)) - 90
        )

        self.test_speed.angle_from_gnd = math.radians(self.test_speed.angle_from_gnd)
        # if not first time

        if self.scaleImu.lasttime != 0:
            # this is the formula for calculating the speed of the submarine
            # vertical speed = ((vertical acceleration - 9.8) * cos(angle between zacc and ground normal)) * time difference

            self.test_speed.z += (
                (-self.scaleImu.zacc - 1)
                * math.cos(self.test_speed.angle_from_gnd)
                * self.scaleImu.timediff
            )

        # print("speed: ", self.test_speed.z) # print the speed for debugging purposes
        self.scaleImu.lasttime = (
            self.scaleImu.sometime
        )  # set the last time to the current time

    def attitudeListener(self, msgType, vehicleAdd, msg):
        self.rotation.pitch = float("{:.4f}".format(math.degrees(msg.pitch)))
        self.rotation.roll = float("{:.4f}".format(math.degrees(msg.roll)))
        self.rotation.yaw = float("{:.4f}".format(math.degrees(msg.yaw)))
        velx = self.vehicle.velocity
        self.test_speed.velx = velx

    def rawImuListener(self, msgType, vehicleAdd, msg):
        self.rawimu.xacc = float("{:.4f}".format(msg.xacc))
        self.rawimu.yacc = float("{:.4f}".format(msg.yacc))
        self.rawimu.zacc = float("{:.4f}".format(msg.zacc))
        self.rawimu.xgyro = float("{:.4f}".format(msg.xgyro))
        self.rawimu.ygyro = float("{:.4f}".format(msg.ygyro))
        self.rawimu.zgyro = float("{:.4f}".format(msg.zgyro))
        self.rawimu.xmag = float("{:.4f}".format(msg.xmag))
        self.rawimu.ymag = float("{:.4f}".format(msg.ymag))
        self.rawimu.zmag = float("{:.4f}".format(msg.zmag))

    def localPositionListener(self, msgType, vehicleAdd, msg):
        self.localPosition.x = float("{:.4f}".format(msg.x))
        self.localPosition.y = float("{:.4f}".format(msg.y))
        self.localPosition.z = float("{:.4f}".format(msg.z))
        self.localPosition.vx = float("{:.4f}".format(msg.vx))
        self.localPosition.vy = float("{:.4f}".format(msg.vy))
        self.localPosition.vz = float("{:.4f}".format(msg.vz))


    def globalPositionListener(self, msgType, vehicleAdd, msg):
        self.globalPosition.time_boot_ms = float("{:.4f}".format(msg.time_boot_ms))
        self.globalPosition.lat = float("{:.4f}".format(msg.lat))
        self.globalPosition.lon = float("{:.4f}".format(msg.lon))
        self.globalPosition.alt = float("{:.4f}".format(msg.alt))
        self.globalPosition.relative_alt = float("{:.4f}".format(msg.relative_alt))
        self.globalPosition.vx = float("{:.4f}".format(msg.vx))
        self.globalPosition.vy = float("{:.4f}".format(msg.vy))
        self.globalPosition.vz = float("{:.4f}".format(msg.vz))
        
        
    def scalePressureListener(self, msgType, vehicleAdd, msg):
        self.scalePressure.press_abs = int(msg.press_abs)
        self.scalePressure.press_diff = int(msg.press_diff)
        self.scalePressure.temperature = int(msg.temperature)

    def arm(self):
        self.vehicle.arm()
        self.vehicle.armed = True

    def disarm(self):
        self.vehicle.disarm()
        self.vehicle.armed = False

    @property  # decorator
    def mode(self) -> str:
        """
        Aracın o anki modunu string olarak döndürür
        ....

        :returns: aracın modu

        """

        return self.vehicle.mode.name

    @mode.setter
    def mode(self, modeName: str):  
        self.vehicle.mode = VehicleMode(modeName.upper())

        
        """

        Aracın modunu değiştirir örnek modlar \n

        -Guided \n

        -Surface \n

        -AltHold \n

        -DepthHold \n
        ....

        :param modeName: aracın geçeceği mod
        """

        print("changed mode to %s" % modeName)

    def SetParameter(self, name, val):
        self.vehicle.parameters.set(name, val)

    def GetParameter(self, name):
        return self.vehicle.parameters.get(name)

    @property
    def Channel1(self):
        return self.vehicle.channels["1"]

    @Channel1.setter
    def Channel1(self, val):
        self.vehicle.channels.overrides["1"] = int(val)

    @property
    def Channel2(self):
        return self.vehicle.channels["2"]

    @Channel2.setter
    def Channel2(self, val):
        self.vehicle.channels.overrides["2"] = int(val)

    @property
    def Channel3(self):
        return self.vehicle.channels["3"]

    @Channel3.setter
    def Channel3(self, val):
        self.vehicle.channels.overrides["3"] = int(val)

    @property
    def Channel4(self):
        return self.vehicle.channels["4"]

    @Channel4.setter
    def Channel4(self, val):
        self.vehicle.channels.overrides["4"] = int(val)

    @property
    def Channel5(self):
        return self.vehicle.channels["5"]

    @Channel5.setter
    def Channel5(self, val):
        self.vehicle.channels.overrides["5"] = int(val)
    @property
    def Channel6(self):
        return self.vehicle.channels["6"]
    @Channel6.setter
    def Channel6(self, val):
        self.vehicle.channels.overrides["6"] = int(val)
    # def setMotorPWM(self, channel, pwm):
    #     try:
    #         if 1000 <= pwm <= 2000:
    #             if self.sim_mode:
    #                 self.vehicle.channels[channel] = int(pwm)
    #                 print(f"Motor {channel} set to {pwm}")
    #             else:
    #                 self.vehicle.channels.overrides[channel] = int(pwm)
    #                 print(f"Motor {channel} set to {pwm}")
    #         else:
    #             print("Invalid PWM value")
    #     except Exception as e:
    #         print(f"Error: {e}")
    # @property
    # def AUX1(self):
    #     if self.sim_mode:
    #         return self.vehicle.channels['5']
    #     return self.vehicle.overrides['1']
    #
    # @AUX1.setter
    # def AUX1(self, val):
    #     if self.sim_mode:
    #         return self.vehicle.channels['5']
    #     self.vehicle.channels.overrides['1'] = int(val)
    #
    # @property
    # def AUX2(self):
    #     if self.sim_mode:
    #         return self.vehicle.channels['6']
    #     return self.vehicle.overrides['2']
    #
    # @AUX2.setter
    # def AUX2(self, val):
    #     if self.sim_mode:
    #         return self.vehicle.channels['6']
    #     self.vehicle.channels.overrides['2'] = int(val)

    def configureAUX(self, numberGPIO):
        """
        Configure the number of AUX pins from 50 to 55 on Pixhawk as GPIOs
        :param numberGPIO: Number of AUX pins to configure

        """
        self.SetParameter("BRD_PWM_COUNT", 6 - numberGPIO)
        print(f"Configured {numberGPIO} AUX pins as GPIOs")
    # BRD_PWM_COUNT = 6 - numberGPIO (0-6)
    # numberGPIO = 3
    # BRD_PWM_COUNT = 6 - 3 = 3
    # 3 AUX pins are configured as GPIOs
    # 50 = RC9 ❌
    # 51 = RC10 ❌
    # 52 = RC11 ❌
    # 53 = Aux 4 ✔
    # 54 = Aux 5 ✔
    # 55 = Aux 6 ✔
    # --------------------------------------------
    # BRD_PWM_COUNT = 6 - numberGPIO (0-6)
    # numberGPIO = 6
    # BRD_PWM_COUNT = 6 - 6 = 0
    # 6 AUX pins are configured as GPIOs
    # 50 = Aux 1 ✔
    # 51 = Aux 2 ✔
    # 52 = Aux 3 ✔
    # 53 = Aux 4 ✔
    # 54 = Aux 5 ✔
    # 55 = Aux 6 ✔
    # Virtual Pins
    def setPWM(self, channel, pwm):
        if 1000 <= pwm <= 2000:
            self.vehicle.channels.overrides[channel] = int(pwm)
            print(f"Channel {channel} set to {pwm}")
        else:
            print("Invalid PWM value")
    # def setServo(self, servo, pwm):
    #     if 1000 <= pwm <= 2000:
    #         self.vehicle.channels.overrides[servo] = int(pwm)
    #         print(f"Servo {servo} set to {pwm}")
    #     else:
    #         print("Invalid PWM value")
    def setServo(self, servo, pwm):
        pwm = int(pwm)
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, servo, pwm, 0, 0, 0, 0, 0        )
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()
    def testChannel(self, test_channel):
        channels = ["1", "2", "3", "4", "5", "6"]
        if test_channel in channels:
            print(f"Testing channel {test_channel}")
            self.vehicle.channels.overrides[test_channel] = 1900
            print(f"Channel {test_channel} set to 1900")
            time.sleep(5)
            self.vehicle.channels.overrides[test_channel] = 1100
            print(f"Channel {test_channel} set to 1100")
            time.sleep(5)
            self.vehicle.channels.overrides[test_channel] = 1500
            print(f"Channel {test_channel} set to 1500")
            time.sleep(5)
        else:
            print("Invalid channel number")

if __name__ == "__main__":
    veh = Submarine("/dev/ttyACM0")
    prev = veh.rotation.yaw
    veh.arm()
    veh.setVehicleModeTo("manual")

    """



    while 1:



        time.sleep(0.1)



        now = veh.rotation.yaw



        if prev != now:



            print("yaw: ", now)




            prev = now



    """
