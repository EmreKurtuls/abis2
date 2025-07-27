import math
import time
import threading
from VehicleMav import Submarine
from AbisTest import PIDController, Autonomous
from experimental_utils.GPS_calculation import calculate_bearing
from geopy.distance import geodesic

class GPSNavigation:
    def __init__(self, vehicle, autonomous_controller):
        self.vehicle = vehicle
        self.auto = autonomous_controller
        
        # Hardcoded coordinates as strings
        self.current_coordinates = "40.712800, -74.006000"  # Example: New York City
        self.target_coordinates = "40.758896, -73.985130"   # Example: Times Square
        
        self.target_lat = None
        self.target_lon = None
        self.start_lat = None
        self.start_lon = None
        self.navigation_active = False
        self.position_tolerance = 0.00001  # GPS coordinate tolerance (~1 meter)
        self.targetDirection_tolerance = 2  # degrees
        
    def parse_coordinates(self, coord_string):
        try:
            lat_str, lon_str = coord_string.split(', ')
            return float(lat_str), float(lon_str)
        except ValueError:
            raise ValueError(f"Invalid coordinate format: {coord_string}. Expected format: 'lat, lon'")
        
    def calculate_distance(self, lat1, lon1, lat2, lon2):
        point1 = (lat1, lon1)
        point2 = (lat2, lon2)
        distance = geodesic(point1, point2).meters
        return distance
        
    def get_current_position(self):
        current_lat, current_lon = self.parse_coordinates(self.current_coordinates)
        return current_lat, current_lon
        
    def set_start_position(self, lat=None, lon=None, use_current=True):
        """Set starting position either manually or from hardcoded coordinates."""
        if use_current:  # Use hardcoded current coordinates
            self.start_lat, self.start_lon = self.get_current_position()
            print(f"Start position set to current location: {self.start_lat:.6f}, {self.start_lon:.6f}")
        elif lat is not None and lon is not None:  # Parameter of function
            self.start_lat = lat
            self.start_lon = lon
            print(f"Start position set manually: {self.start_lat:.6f}, {self.start_lon:.6f}")
        else:
            raise ValueError("Either use_current=True or provide lat/lon coordinates")
            
    def set_target_position(self, lat=None, lon=None, use_hardcoded=True):
        """Set target GPS coordinates from hardcoded string or manual input."""
        if use_hardcoded: # Use hardcoded target coordinates
            self.target_lat, self.target_lon = self.parse_coordinates(self.target_coordinates)
            print(f"Target position set from hardcoded: {self.target_lat:.6f}, {self.target_lon:.6f}")
        elif lat is not None and lon is not None: # Parameter of function
            self.target_lat = lat
            self.target_lon = lon
            print(f"Target position set manually: {self.target_lat:.6f}, {self.target_lon:.6f}")
        else:
            raise ValueError("Either use_hardcoded=True or provide lat/lon coordinates")
        
    def navigate_to_target(self, max_speed=400, navigation_timeout=300):
        if self.target_lat is None or self.target_lon is None:
            print("Error: Target position not set!")
            return False
            
        if self.start_lat is None or self.start_lon is None:
            print("Error: Start position not set!")
            return False
            
        print("Starting GPS navigation...")
        print(f"From: {self.start_lat:.6f}, {self.start_lon:.6f}")
        print(f"To: {self.target_lat:.6f}, {self.target_lon:.6f}")
        
        # Calculate initial distance using geopy
        initial_distance = self.calculate_distance(
            self.start_lat, self.start_lon,
            self.target_lat, self.target_lon
        )
        print(f"Initial distance to target: {initial_distance:.2f} meters")
        
        self.navigation_active = True
        start_time = time.time()
        
        # Start stabilization threads
        pitch_thread = threading.Thread(target=self.auto.stablePitch, daemon=True)
        roll_thread = threading.Thread(target=self.auto.stableRoll, daemon=True)
        altitude_thread = threading.Thread(target=self.auto.stableAltitude, daemon=True)
        
        pitch_thread.start()
        roll_thread.start()
        altitude_thread.start()
        
        try:
            while self.navigation_active and (time.time() - start_time) < navigation_timeout:
                # Get current position (in real scenario, this would be from vehicle sensors)
                current_lat, current_lon = self.get_current_position()
                
                # Calculate distance to target using geopy
                distance_to_target = self.calculate_distance(
                    current_lat, current_lon, 
                    self.target_lat, self.target_lon
                )
                
                print(f"Current position: {current_lat:.6f}, {current_lon:.6f}")
                print(f"Distance to target: {distance_to_target:.2f} meters")
                
                # Check if we've reached the target
                if distance_to_target < 5.0:  # Within 5 meters
                    print("Target reached!")
                    self.navigation_active = False
                    break
                    
                # Calculate targetDirection to target
                target_targetDirection = calculate_bearing(
                    current_lat, current_lon,
                    self.target_lat, self.target_lon
                )
                
                print(f"Target targetDirection: {target_targetDirection:.1f} degrees")
                
                # Get current heading
                current_heading = self.vehicle.rotation.yaw
                if current_heading < 0:
                    current_heading += 360
                    
                # Calculate heading error
                heading_error = target_targetDirection - current_heading
                if heading_error > 180:
                    heading_error -= 360
                elif heading_error < -180:
                    heading_error += 360
                    
                print(f"Current heading: {current_heading:.1f}, Heading error: {heading_error:.1f}")
                
                # Adjust heading if necessary
                if abs(heading_error) > self.targetDirection_tolerance:
                    print(f"Adjusting heading by {heading_error:.1f} degrees")
                    new_target_yaw = current_heading + heading_error
                    if new_target_yaw < 0:
                        new_target_yaw += 360
                    elif new_target_yaw >= 360:
                        new_target_yaw -= 360
                        
                    self.auto.stableYaw(target_yaw=new_target_yaw)
                    time.sleep(1)  # Wait for heading adjustment
                else:
                    # Move forward towards target
                    speed = min(max_speed, max(100, int(distance_to_target * 2)))  # Adaptive speed
                    print(f"Moving forward with speed: {speed}")
                    self.auto.moveForward(intended_time=2, signal_multiplier=speed)
                    
                time.sleep(0.5)  # Navigation update rate
                
        except KeyboardInterrupt:
            print("Navigation interrupted by user")
            self.navigation_active = False
        except Exception as e:
            print(f"Navigation error: {e}")
            self.navigation_active = False
        finally:
            # Stop the vehicle
            self.auto.STOP()
            print("Navigation completed")
            
        return True
        
    def stop_navigation(self):
        """Stop the current navigation."""
        self.navigation_active = False
        self.auto.STOP()
        print("Navigation stopped")


def test_gps_navigation():
    """Test function for GPS navigation."""
    try:
        # Initialize vehicle
        rov = Submarine('/dev/ttyACM0') # Adjust USB port
        rov.setVehicleModeTo("manual")
        time.sleep(1)
        
        if not rov.arm():
            print("Failed to arm vehicle!")
            return
            
        print("Vehicle armed successfully")
        
        # Initialize PID controllers
        pid_pitch = PIDController(kp=1.5, ki=0.4, kd=0.05, windup_guard=20, ramp_rate=5, filter_coefficient=0.5)
        pid_roll = PIDController(kp=3, ki=0.4, kd=0.03, windup_guard=15, ramp_rate=5, filter_coefficient=0.5)
        pid_altitude = PIDController(kp=2, ki=0.4, kd=0.05, windup_guard=30, ramp_rate=5, filter_coefficient=0.5)
        pid_yaw = PIDController(kp=10, ki=0.4, kd=0.1, windup_guard=15, ramp_rate=5, filter_coefficient=0.5)
        
        # Initialize autonomous controller
        auto = Autonomous(rov, pid_pitch, pid_roll, pid_altitude, pid_yaw)
        auto.initialization()
        
        # Initialize GPS navigation
        gps_nav = GPSNavigation(rov, auto)
        
        # Set start position (use hardcoded current coordinates)
        gps_nav.set_start_position(use_current=True)
        
        # Set target position (use hardcoded target coordinates)
        gps_nav.set_target_position(use_hardcoded=True)
        
        # Start navigation
        gps_nav.navigate_to_target(max_speed=250, navigation_timeout=600)
        
    except Exception as e:
        print(f"Test error: {e}")
    finally:
        if 'rov' in locals():
            rov.close()


if __name__ == "__main__":
    test_gps_navigation()
