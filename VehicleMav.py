import random
import datetime
import math
import time
from pymavlink import mavutil


class Submarine:

    vehicle = None
    somenumberforme = 0
    _channels = {1: 1500, 2: 1500, 3: 1500, 4: 1500, 5: 1500, 6: 1500}

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
        hdg = 0

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

    def __init__(self, Usb="COM4", ip="127.0.0.1", port=5760):
        try:
            if Usb:
                print(f"Connecting to vehicle at {Usb}")
                self.vehicle = mavutil.mavlink_connection(Usb, baud=38400)
            else:
                print(f"Connecting to vehicle at tcp:{ip}:{port}")
                self.vehicle = mavutil.mavlink_connection(f"tcp:{ip}:{port}")
            print("Waiting for vehicle heartbeat")
            self.vehicle.wait_heartbeat()
            print("Successfully connected to vehicle")
        except Exception as e:
            print("Error: ", e)
            self.vehicle = None
            return

    def setVehicleModeTo(self, modeName: str, Force=False):
        if self.vehicle is not None:
            while True:
                current_mode = self.vehicle.flightmode
                print(f"Current mode: {current_mode}")
                if current_mode != modeName.upper() or Force:
                    try:
                        self.vehicle.set_mode(modeName.upper())
                        print(f"Changed mode to {modeName.upper()}")
                        Force = True
                        break
                    except Exception as e:
                        print(f"Error setting mode: {e}")
                        break
                else:
                    print(f"Already in {modeName.upper()} mode")
                    break
        else:
            print("Not Connected")

    def getVehicleMode(self):
        try:
            if self.vehicle is not None:
                mode = self.vehicle.flightmode
                print("Vehicle Mode: ", mode)
                return str(mode)
            else:
                return "Not Connected"
        except Exception as e:
            print("Error: ", e)
            return "Error"

    def scaledImu2Listener(self, msg):
        self.scaleImu.sometime = msg.time_boot_ms / 100.0
        self.scaleImu.timediff = self.scaleImu.sometime - self.scaleImu.lasttime
        self.scaleImu.xacc = msg.xacc / 1000.0
        self.scaleImu.yacc = msg.yacc / 1000.0
        self.scaleImu.zacc = msg.zacc / 1000.0
        self.scaleImu.xgyro = msg.xgyro / 1000.0
        self.scaleImu.ygyro = msg.ygyro / 1000.0
        self.scaleImu.zgyro = msg.zgyro / 1000.0

        self.test_speed.angle_from_gnd = math.atan2(self.scaleImu.zacc,
                                                    math.sqrt((self.scaleImu.yacc ** 2) + (self.scaleImu.xacc ** 2)))
        self.test_speed.angle_from_gnd = abs(abs(math.degrees(self.test_speed.angle_from_gnd)) - 90)
        self.test_speed.angle_from_gnd = math.radians(self.test_speed.angle_from_gnd)

        if self.scaleImu.lasttime != 0:
            self.test_speed.z += (-self.scaleImu.zacc - 1) * math.cos(
                self.test_speed.angle_from_gnd) * self.scaleImu.timediff

        self.scaleImu.lasttime = self.scaleImu.sometime

    def attitudeListener(self, msg):
        self.rotation.pitch = math.degrees(msg.pitch)
        self.rotation.roll = math.degrees(msg.roll)
        self.rotation.yaw = math.degrees(msg.yaw)
        velx = self.vehicle.velocity
        self.test_speed.velx = velx

    def rawImuListener(self, msg):
        self.rawimu.xacc = msg.xacc
        self.rawimu.yacc = msg.yacc
        self.rawimu.zacc = msg.zacc
        self.rawimu.xgyro = msg.xgyro
        self.rawimu.ygyro = msg.ygyro
        self.rawimu.zgyro = msg.zgyro
        self.rawimu.xmag = msg.xmag
        self.rawimu.ymag = msg.ymag
        self.rawimu.zmag = msg.zmag

    def localPositionListener(self, msg):
        self.localPosition.x = msg.x
        self.localPosition.y = msg.y
        self.localPosition.z = msg.z
        self.localPosition.vx = msg.vx
        self.localPosition.vy = msg.vy
        self.localPosition.vz = msg.vz

    def globalPositionListener(self, msg):
        self.globalPosition.x = msg.x
        self.globalPosition.y = msg.y
        self.globalPosition.z = msg.z
        self.globalPosition.vx = msg.vx
        self.globalPosition.vy = msg.vy
        self.globalPosition.vz = msg.vz

    def scalePressureListener(self, msg):
        self.scalePressure.press_abs = msg.press_abs
        self.scalePressure.press_diff = msg.press_diff
        self.scalePressure.temperature = msg.temperature

    def arm(self):
        if self.vehicle is not None:
            self.vehicle.arducopter_arm()
            self.vehicle.motors_armed_wait()
        else:
            print("Not Connected")

    def disarm(self):
        if self.vehicle is not None:
            self.vehicle.arducopter_disarm()
            self.vehicle.motors_disarmed_wait()
        else:
            print("Not Connected")

    @property
    def mode(self) -> str:
        return self.vehicle.flightmode if self.vehicle is not None else "Not Connected"

    @mode.setter
    def mode(self, modeName: str):
        if self.vehicle is not None:
            self.vehicle.set_mode(modeName.upper())
            print("changed mode to %s" % modeName)
        else:
            print("Not Connected")

    def SetParameter(self, name, val):
        if self.vehicle is not None:
            self.vehicle.param_set_send(name, val)
        else:
            print("Not Connected")

    def GetParameter(self, name):
        if self.vehicle is not None:
            return self.vehicle.param_fetch_one(name)
        else:
            print("Not Connected")
            return None


    def _send_rc_override(self):
        # Send RC override to the Pixhawk for all channels
        self.vehicle.mav.rc_channels_override_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            self._channels[1],  # Channel 1
            self._channels[2],  # Channel 2
            self._channels[3],  # Channel 3
            self._channels[4],  # Channel 4
            self._channels[5],
            self._channels[6],
            0, 0, 0  # Other channels (if any)
        )

    def Channel1(self):
        return self._channels[1]

    
    def Channel1(self, val):
        # Ensure the value is within valid PWM range
        self._channels[1] = int(min(max(val, 1100), 1900))  # Clamp between 1000 and 2000
        self._send_rc_override()
        print(f"Channel 1 set to {val}")

    @property
    def Channel2(self):
        return self._channels[2]

    @Channel2.setter
    def Channel2(self, val):
        self._channels[2] = int(min(max(val, 1100), 1900))  # Clamp between 1000 and 2000
        self._send_rc_override()
        print(f"Channel 2 set to {val}")

    @property
    def Channel3(self):
        return self._channels[3]

    @Channel3.setter
    def Channel3(self, val):
        self._channels[3] = int(min(max(val, 1100), 1900))  # Clamp between 1000 and 2000
        self._send_rc_override()
        print(f"Channel 3 set to {val}")

    @property
    def Channel4(self):
        return self._channels[4]

    @Channel4.setter
    def Channel4(self, val):
        self._channels[4] = int(min(max(val, 1100), 1900))  # Clamp between 1000 and 2000
        self._send_rc_override()
        print(f"Channel 4 set to {val}")

    @property
    def Channel5(self):
        return self._channels[5]
    @Channel5.setter
    def Channel5(self, val):
        self._channels[5] = int(min(max(val, 1100), 1900))  # Clamp between 1000 and 2000
        self._send_rc_override()
        print(f"Channel 5 set to {val}")

    @property
    def Channel6(self):
        return self._channels[6]

    @Channel6.setter
    def Channel6(self, val):
        self._channels[6] = int(min(max(val, 1100), 1900))  # Clamp between 1000 and 2000
        self._send_rc_override()
        print(f"Channel 6 set to {val}")

    def setPWM(self, channel, pwm):
        if self.vehicle is not None:
            if 1000 <= pwm <= 2000:
                self.vehicle.channels.overrides[channel] = int(pwm)
                print(f"Channel {channel} set to {pwm}")
            else:
                print("Invalid PWM value")
        else:
            print("Not Connected")

    def setServo(self, servo, pwm):
        if self.vehicle is not None:
            pwm = int(pwm)
            msg = self.vehicle.message_factory.command_long_encode(
                0, 0,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,
                servo,
                pwm,
                0, 0, 0, 0, 0
            )
            self.vehicle.send_mavlink(msg)
            self.vehicle.flush()
        else:
            print("Not Connected")

    def testChannel(self, test_channel):
        if self.vehicle is not None:
            channels = ['1', '2', '3', '4', '5', '6']
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
        else:
            print("Not Connected")


if __name__ == "__main__":
    veh = Submarine(Usb="/dev/ttyACM0")  # Modify USB port as needed for your setup
    prev = veh.rotation.yaw
    veh.arm()
    veh.setVehicleModeTo("manual")

    alt = veh.globalPosition.alt

    print('alt: ', alt)

    '''
    while 1:
        time.sleep(0.1)
        now = veh.rotation.yaw
        if prev != now:
            print("yaw: ", now)
            prev = now
    '''
