from pymavlink import mavutil
import time

# Create MAVLink connection (adjust port and baud)
# macOS typical Pixhawk serial port: /dev/tty.usbmodemXXXXX or /dev/cu.usbmodemXXXX
master = mavutil.mavlink_connection('/dev/cu.usbmodem1101', baud=57600)

# Wait for heartbeat from Pixhawk
print("Waiting for heartbeat...")
master.wait_heartbeat()
print("✅ Heartbeat received.")

# Request specific message streams
print("Requesting data streams...")

# Request GPS data stream
master.mav.request_data_stream_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_POSITION,
    2,  # Hz
    1   # Start
)

# Request attitude data stream
master.mav.request_data_stream_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
    10,  # Hz - higher rate for attitude
    1   # Start
)

# Also request all data as backup
master.mav.request_data_stream_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,
    2,  # Hz
    1   # Start
)

print("Data streams requested.")

# Read GPS and attitude data
import math

# Initialize attitude variables
current_yaw = 0.0
current_roll = 0.0
current_pitch = 0.0
message_count = 0
attitude_count = 0
gps_count = 0
last_gps_print = 0
last_attitude_print = 0
message_types = {}

print("Starting to listen for messages...")

while True:
    # Check for any message to see what types we're getting
    msg = master.recv_match(blocking=False)
    if msg:
        message_count += 1
        msg_type = msg.get_type()
        
        # Count message types
        if msg_type in message_types:
            message_types[msg_type] += 1
        else:
            message_types[msg_type] = 1
        
        # Print message type summary every 500 messages
        if message_count % 500 == 0:
            print(f"\n--- Message Summary (Total: {message_count}) ---")
            for mtype, count in sorted(message_types.items()):
                print(f"{mtype}: {count}")
            print("---")
        
        # Handle specific message types
        if msg_type == 'GPS_RAW_INT':
            gps_count += 1
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.alt / 1000.0
            fix_type = msg.fix_type

            # Only print GPS every 2 messages to reduce spam
            if gps_count % 2 == 0:
                print(msg)
                # print(f"GPS #{gps_count} - Lat: {lat:.6f}, Lon: {lon:.6f}, Alt: {alt:.2f}m, Fix: {fix_type}, Sats: {msg.satellites_visible}")
        
        elif msg_type == 'ATTITUDE':
            attitude_count += 1
            current_pitch = math.degrees(msg.pitch)
            current_roll = math.degrees(msg.roll)
            current_yaw = math.degrees(msg.yaw)
            
            # print(f"ATTITUDE #{attitude_count} - Pitch: {current_pitch:.2f}°, Roll: {current_roll:.2f}°, Yaw: {current_yaw:.2f}°")
    
    time.sleep(0.01)  # Faster polling