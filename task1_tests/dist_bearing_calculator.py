import math
from geopy.distance import geodesic

def calculate_bearing(lat1, lon1, lat2, lon2):
    """
    Calculate the initial bearing (in degrees) from (lat1, lon1) to (lat2, lon2).
    This uses the spherical Earth model, which might be less precise for very short distances
    compared to ellipsoidal models, but is generally good for bearing.
    
    Args:
        lat1, lon1: Latitude and longitude of the starting point in decimal degrees.
        lat2, lon2: Latitude and longitude of the destination point in decimal degrees.
    
    Returns:
        Bearing in degrees from North (0° to 360°).
    """
    # Convert from degrees to radians
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    diff_long = math.radians(lon2 - lon1)

    # Bearing calculation
    x = math.sin(diff_long) * math.cos(lat2_rad)
    y = math.cos(lat1_rad) * math.sin(lat2_rad) - \
        math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(diff_long)

    initial_bearing = math.atan2(x, y)

    # Convert from radians to degrees and normalize
    initial_bearing_deg = (math.degrees(initial_bearing) + 360) % 360
    return initial_bearing_deg

# For distance, we'll use geopy for higher precision
# def calculate_distance(lat1, lon1, lat2, lon2):
#     # This function will be replaced by geopy.distance.geodesic
#     pass


import csv
import os

def log_test_result(case_id, description, lat1, lon1, lat2, lon2, expected_distance, expected_bearing, calc_distance, calc_bearing):
    print(f"Case: {case_id} - {description}")
    print(f"\n\t\tStart: ({lat1}, {lon1}) \t\tEnd: ({lat2}, {lon2})\n")
    if expected_distance is not None and expected_bearing is not None:
        print(f"  expected:    {expected_distance} m\t\texpected:    {expected_bearing}°")
        print(f"  calculated:  {calc_distance:.4f} m\t\tcalculated:  {calc_bearing:.4f}°")
        print(f"  err:         {abs(calc_distance - expected_distance):.4f} m\t\terr:         {abs(calc_bearing - expected_bearing):.4f}°")
    elif expected_distance is not None:
        print(f"  expected:    {expected_distance} m")
        print(f"  calculated:  {calc_distance:.4f} m")
        print(f"  err:         {abs(calc_distance - expected_distance):.4f} m")
        print(f"  calculated:  {calc_bearing:.4f}°")
    elif expected_bearing is not None:
        print(f"  expected:    {expected_bearing}°")
        print(f"  calculated:  {calc_bearing:.4f}°")
        print(f"  err:         {abs(calc_bearing - expected_bearing):.4f}°")
        print(f"  calculated:  {calc_distance:.4f} m")
    else:
        print(f"  calculated:  {calc_distance:.4f} m\tcalculated:  {calc_bearing:.4f}°")
    print("-" * 80)

if __name__ == "__main__":
    csv_path = os.path.join(os.path.dirname(__file__), "test_data_GPS.csv")
    with open(csv_path, newline="") as csvfile:
        reader = csv.DictReader(csvfile)
        print("--- Geodesic Distance and Bearing Test Results ---")
        print("-" * 80)
        for row in reader:
            lat1 = float(row["lat1"])
            lon1 = float(row["lon1"])
            lat2 = float(row["lat2"])
            lon2 = float(row["lon2"])
            expected_distance = float(row["expected_distance_m"]) if row["expected_distance_m"] else None
            expected_bearing = float(row["expected_initial_bearing_deg"]) if row["expected_initial_bearing_deg"] else None

            calc_distance = geodesic((lat1, lon1), (lat2, lon2)).meters
            calc_bearing = calculate_bearing(lat1, lon1, lat2, lon2)

            log_test_result(
                row["case_id"],
                row["description"],
                lat1, lon1, lat2, lon2,
                expected_distance, expected_bearing,
                calc_distance, calc_bearing
            )

