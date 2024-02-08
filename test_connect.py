from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math
import argparse

parser = argparse.ArgumentParser(description='commands')
parser.add_argument('--connect')
args = parser.parse_args()

connection_string = args.connect


print("Connection to the vehicle on %s" % connection_string)
vehicle = connect(connection_string, wait_ready=True)


def arm_and_takeoff(tgt_altitude):
    print("Arming motors")
    
    while not vehicle.is_armable:
        time.sleep(1)
        
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    
    while not vehicle.armed: 
        time.sleep(1)
    
    print("Takeoff")
    vehicle.simple_takeoff(tgt_altitude)
    
    while True:
        altitude = vehicle.location.global_relative_frame.alt
        
        if altitude >= tgt_altitude - 1:
            print("Altitude reached")
            break
            
        time.sleep(1)
        

def move_forward(distance):
    print("Moving forward")
    current_location = vehicle.location.global_relative_frame
    target_location = get_location_metres(current_location, 0, distance)
    vehicle.simple_goto(target_location)
    while True:
        remaining_distance = get_distance_metres(vehicle.location.global_relative_frame, target_location)
        if remaining_distance <= 0.5:  # Threshold distance to consider as reached
            print("Target position reached")
            break
        time.sleep(1)
        

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned Location has the same `alt` value as `original_location`.
    """
    earth_radius = 6378137.0  # Radius of the earth in meters
    # Coordinate offsets in radians
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))

    # New position in decimal degrees
    newlat = original_location.lat + (dLat * 180 / math.pi)
    newlon = original_location.lon + (dLon * 180 / math.pi)
    return LocationGlobalRelative(newlat, newlon, original_location.alt)


def get_distance_metres(location1, location2):
    """
    Returns the ground distance in meters between two `LocationGlobal` objects.
    This method is an approximation and assumes the earth is a sphere.
    """
    dlat = location2.lat - location1.lat
    dlong = location2.lon - location1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5
    

# ------ MAIN PROGRAM ----
arm_and_takeoff(2)  # Takeoff to 2 meters altitude
time.sleep(5)  # Give some time to stabilize

move_forward(8)  # Move forward 8 meters
time.sleep(5)  # Give some time to reach the destination

# Return to the original location
print("Returning to the original location")
vehicle.mode = VehicleMode("RTL")
while vehicle.mode.name != "RTL":  # Wait for the mode change
    time.sleep(1)

time.sleep(10)  # Give some time to return

vehicle.close()  # Close connection
