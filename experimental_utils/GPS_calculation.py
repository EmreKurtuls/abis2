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

if __name__ == "__main__":
    # Define beginning coordinates
    beginning_coords = {
        '32': (40.991827, 28.832032),
        '45': (40.991813, 28.832145),
        '45*': (40.991822, 28.832045),
        '57': (40.991767, 28.832057)
    }

    # Define ending coordinates
    ending_coords = {
        '97': (40.992130, 28.832092),
        '22': (40.992082, 28.832122),
        '80': (40.992107, 28.832080),
        '83': (40.992100, 28.832083) 
    }

    print("--- Geodesic Distances and Bearings for all Combinations ---")
    print("-" * 60)

    # Iterate through every combination of beginning and ending coordinates
    for begin_name, (lat1, lon1) in beginning_coords.items():
        for end_name, (lat2, lon2) in ending_coords.items():
            coords_1 = (lat1, lon1)
            coords_2 = (lat2, lon2)

            # Calculate geodesic distance
            distance_geodesic = geodesic(coords_1, coords_2).meters

            # Calculate bearing
            bearing = calculate_bearing(lat1, lon1, lat2, lon2)

            print(f"Combination: '{begin_name}-{end_name}'")
            # print(f"  Start: ({lat1:.6f}, {lon1:.6f})")
            # print(f"  End:   ({lat2:.6f}, {lon2:.6f})")
            print(f"  Geodesic Distance (Vincenty): {distance_geodesic:.4f} meters")
            print(f"  Bearing (Spherical Model): {bearing:.4f}°")
            print("-" * 60)

