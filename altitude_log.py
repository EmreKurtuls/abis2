from pymavlink import mavutil
import time
import csv
import os

def setup_csv_logger(filename=None):
    
    filename = 'AbisRepo2025/altitude_log3.csv'
    
    fieldnames = ['Relative_Alt_m', 'Pressure_Pa']
    
    file_exists = os.path.isfile(filename)
    with open(filename, 'a', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        if not file_exists:
            writer.writeheader()
    
    print(f"Logging data to {filename}")
    return filename, fieldnames

def log_data_to_csv(filename, fieldnames, data):

    with open(filename, 'a', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writerow(data)

def altitude_to_pressure(altitude_meters):
    
    P0 = 101325  # Sea level standard atmospheric pressure in Pa
    T0 = 288.15  # Sea level standard temperature in K
    g = 9.80665  # Gravitational acceleration in m/s²
    L = 0.0065   # Temperature lapse rate in K/m
    R = 287.05   # Specific gas constant for dry air in J/(kg·K)
    
    # Barometric formula for the troposphere
    pressure_pa = P0 * (1 - (L * altitude_meters) / T0) ** (g / (R * L))
    
    return round(pressure_pa, 2)

def main():
    # Connect to the SITL instance
    sub = mavutil.mavlink_connection('/dev/ttyACM0')
    
    print("Waiting for heartbeat...")
    sub.wait_heartbeat()
    print("Heartbeat received! Connection successful.")
    
    sub.mav.request_data_stream_send(
        sub.target_system,
        sub.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        10,  # 10 Hz
        1    # Start sending
    )
    
    # Set up CSV logger
    csv_filename, fieldnames = setup_csv_logger()
    
    print("time.sleep(30)")
    # time.sleep(30)
    while True:
        # Clear the previous data by receiving all pending messages
        while sub.recv_match(blocking=False) is not None:
            pass
        
        # Get the current time
        
            
        # Initialize data dictionary
        data = {
            'Relative_Alt_m': None,
            'Pressure_Pa': None
        }
            
        # Get the latest altitude message
        msg = sub.recv_match(
            type=['GLOBAL_POSITION_INT'],
            blocking=True,
            timeout=1
        )
            
        if msg is not None:
            rel_alt = msg.relative_alt / 1000.0
            alt_pascal = altitude_to_pressure(rel_alt)
            
            print('msg: ', msg)
            
            data['Relative_Alt_m'] = rel_alt
            data['Pressure_Pa'] = alt_pascal
                
            # print(f"Relative Alt: {rel_alt:.2f}m")
            # print(f'Pascal: {alt_pascal:.2f} Pa')
                
            # Log data to CSV
            log_data_to_csv(csv_filename, fieldnames, data)
        else:
            print(f"No altitude data received")
            
        time.sleep(0.5)
    


if __name__ == "__main__":
    main()