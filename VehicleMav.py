import random
import datetime
import math
import time
from pymavlink import mavutil
import threading

class Submarine:

    vehicle = None
    somenumberforme = 0
    _channels = [1500] * 8

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

    class scalePressure2:
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
        self._is_rc_override_active = False
        self._rc_override_thread = None
        self._is_message_listener_active = False
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
            self.mav = self.vehicle.mav
            self.target_system = self.vehicle.target_system
            self.target_component = self.vehicle.target_component

            self._start_rc_override()
            self._start_message_listener()

        except Exception as e:
            print("Error: ", e)
            self.vehicle = None
            return

    def _message_listener_worker(self):
        print("Message listener thread started.")
        while self._is_message_listener_active:
            try:
                msg = self.vehicle.recv_match(blocking=True, timeout=1)
                if msg is None:
                    continue
                
                msg_type = msg.get_type()
                
                if msg_type == 'ATTITUDE':
                    self.attitudeListener(msg)
                elif msg_type == 'GLOBAL_POSITION_INT':
                    self.globalPositionListener(msg)
                elif msg_type == 'LOCAL_POSITION_NED':
                    self.localPositionListener(msg)
                elif msg_type == 'SCALED_PRESSURE2':
                    self.scalePressureListener(msg)

            except Exception as e:
                print(f"Message listener error: {e}")
                self._is_message_listener_active = False
                break
        print("Message listener thread stopped.")

    def _start_message_listener(self):
        if self.vehicle and not self._is_message_listener_active:
            self._is_message_listener_active = True
            self._message_listener_thread = threading.Thread(target=self._message_listener_worker, daemon=True)
            self._message_listener_thread.start()


    def _rc_override_worker(self):
        while self._is_rc_override_active:
            try:
                self.vehicle.mav.rc_channels_override_send(
                    self.vehicle.target_system,
                    self.vehicle.target_component,
                    *self._channels 
                )
                time.sleep(1 / 20)
            except Exception as e:
                print(f"RC override worker error: {e}")
                self._is_rc_override_active = False
                break
        print("RC Override thread stopped.")


    def _start_rc_override(self):
        if self.vehicle and not self._is_rc_override_active:
            self._channels = [1500, 1500, 1500, 1500, 1500, 1500, 0, 0] 
            self._is_rc_override_active = True
            self._rc_override_thread = threading.Thread(target=self._rc_override_worker, daemon=True)
            self._rc_override_thread.start()
            print("RC Override thread started.")

    def close(self):
        print("Closing vehicle connection...")

        if self._is_message_listener_active:
            self._is_message_listener_active = False
            if hasattr(self, '_message_listener_thread'):
                self._message_listener_thread.join(timeout=1)

        if self._is_rc_override_active:
            self._is_rc_override_active = False
            if self._rc_override_thread:
                self._rc_override_thread.join(timeout=1)

        if self.vehicle and self.vehicle.port.is_open:
            try:
                self.relinquish_control()
                self.disarm()
            except Exception as e:
                print(f"Could not send disarm command during close: {e}")

        if self.vehicle:
            self.vehicle.close()

        print("Connection closed.")


    def relinquish_control(self):
        print("Relinquishing RC control...")
        self._channels = [0] * 8
        time.sleep(0.5)


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
        self.globalPosition.time_boot_ms = msg.time_boot_ms
        self.globalPosition.lat = msg.lat  # .x yerine .lat
        self.globalPosition.lon = msg.lon  # .y yerine .lon
        self.globalPosition.alt = msg.alt  # .z yerine .alt
        # İrtifa kontrolü için en önemli veri:
        self.globalPosition.relative_alt = msg.relative_alt / 1000.0  # milimetreden metreye çevir
        self.globalPosition.vx = msg.vx
        self.globalPosition.vy = msg.vy
        self.globalPosition.vz = msg.vz
        self.globalPosition.hdg = msg.hdg

    def scalePressureListener(self, msg):
        self.scalePressure2.press_abs = msg.press_abs
        self.scalePressure2.press_diff = msg.press_diff
        self.scalePressure2.temperature = msg.temperature

    def arm(self):
        self.mav.command_long_send(
            self.target_system, self.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        ack = self.vehicle.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("Araç arm edildi.")
            # self.motors_armed_wait() 
            print("Motorlar aktif.")
            return True
        else:
            print("arm başarısız!")
            return False

    def disarm(self):
        self.mav.command_long_send(
            self.target_system, self.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        ack = self.vehicle.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("Araç disarm edildi.")
        else:
            print("disarm başarısız.")

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


    def relinquish_control(self):
        print("Tüm kanallarda kontrol Pixhawk'a devrediliyor...")
        channels = [0] * 8
        for _ in range(int(10)):
            self.mav.rc_channels_override_send(
                self.target_system, self.target_component, *channels
            )
            time.sleep(1 / 10)

    @property
    def Channel1(self): return self._channels[0]
    @Channel1.setter
    def Channel1(self, val): self._channels[0] = int(min(max(val, 1100), 1900))

    @property
    def Channel2(self): return self._channels[1]
    @Channel2.setter
    def Channel2(self, val): self._channels[1] = int(min(max(val, 1100), 1900))
    
    @property
    def Channel3(self): return self._channels[2]
    @Channel3.setter
    def Channel3(self, val): self._channels[2] = int(min(max(val, 1100), 1900))

    @property
    def Channel4(self): return self._channels[3]
    @Channel4.setter
    def Channel4(self, val): self._channels[3] = int(min(max(val, 1100), 1900))

    @property
    def Channel5(self): return self._channels[4]
    @Channel5.setter
    def Channel5(self, val): self._channels[4] = int(min(max(val, 1100), 1900))

    @property
    def Channel6(self): return self._channels[5]
    @Channel6.setter
    def Channel6(self, val): self._channels[5] = int(min(max(val, 1100), 1900))

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
    veh = Submarine(Usb="/dev/cu.usbmodem11101")  # Modify USB port as needed for your setup
    prev = veh.rotation.yaw
    veh.arm()
    veh.setVehicleModeTo("manual")

    alt = veh.scalePressure2.press_abs

    print('alt: ', alt)

    '''
    while 1:
        time.sleep(0.1)
        now = veh.rotation.yaw
        if prev != now:
            print("yaw: ", now)
            prev = now
    '''

