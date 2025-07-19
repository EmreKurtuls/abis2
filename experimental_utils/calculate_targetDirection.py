import math

def calculate_bearing(lat1, lon1, lat2, lon2):
    """
    Calculate the initial targetDirection (in degrees) from (lat1, lon1) to (lat2, lon2).
    
    Args:
        lat1, lon1: Latitude and longitude of the starting point in decimal degrees.
        lat2, lon2: Latitude and longitude of the destination point in decimal degrees.
    
    Returns:
        targetDirection in degrees from North (0° to 360°).
    """
    # Convert from degrees to radians
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    diff_long = math.radians(lon2 - lon1)

    # targetDirection calculation
    x = math.sin(diff_long) * math.cos(lat2_rad)
    y = math.cos(lat1_rad) * math.sin(lat2_rad) - \
        math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(diff_long)

    initial_targetDirection = math.atan2(x, y)

    # Convert from radians to degrees and normalize
    initial_targetDirection_deg = (math.degrees(initial_targetDirection) + 360) % 360
    return initial_targetDirection_deg