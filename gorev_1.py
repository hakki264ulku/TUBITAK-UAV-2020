from dronekit import connect, LocationGlobalRelative, mavutil, VehicleMode
import time, math

#CONNECT
connection_string = "/dev/ttyAMA0" #(also set baud=57600) use this baud rate in the pixhawk
iha = None
connectTrial = 0
maxTrial = 3
baud_rate = 921600

while connectTrial < maxTrial:
    try:
        iha = connect(connection_string, baud=baud_rate, wait_ready=True)
        break
    except:
        connectTrial +=1
        time.sleep(1)

    if connectTrial == maxTrial:
        print "exiting..."
        exit()

def arm_and_takeOff(targetAltitude):
    print "Basic pre-arm checks"
    # Don't try to arm until autopilot is ready
    while not iha.is_armable:
        print " Waiting for iha to initialise..."
        time.sleep(1)


    print "Arming motors"
    # Copter should arm in GUIDED mode (this mode is recomended for autonomous flight)
    iha.mode = VehicleMode("GUIDED")
    iha.armed = True

    time.sleep(3)

    print "Arming motors"
    iha.armed = True

    while not iha.armed:
        print "waiting for the arming"
        iha.armed = True
        time.sleep(1)
    
    #If the code has come up to this point, it is then ready to take off

    print "waiting for 5 seconds"
    time.sleep(5)

    print "start taking off up to", targetAltitude, " meters"
    iha.simple_takeoff(targetAltitude)

    while True:
        print "Altitude: ", iha.location.global_relative_frame.alt
        if iha.location.global_relative_frame.alt >= targetAltitude*0.95:
            print "reached target altitude!"
            break
        time.sleep(1) 

def get_distance_metres(aLocation1, aLocation2):

    """
    Returns the ground distance in metres between two `LocationGlobal` or `LocationGlobalRelative` objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


# arm and takeoff up to 4 meters, all required checks are done inside the function
height = 4
arm_and_takeOff(height)

time.sleep(3) # hover

homePoint = iha.location.global_relative_frame #take it as home point during the hover

waypoints = []

wp1 = LocationGlobalRelative(37.8750293, 32.4751072, height) # args = (lat, lon, alt)
waypoints.append(wp1)

wp2 = LocationGlobalRelative(37.8751049, 32.4750661, height) # args = (lat, lon, alt)
waypoints.append(wp2)

wp3 = LocationGlobalRelative(37.8751415, 32.4751619, height) # args = (lat, lon, alt)
waypoints.append(wp3)

wp4 = LocationGlobalRelative(37.8750257, 32.4751778, height) # args = (lat, lon, alt)
waypoints.append(wp4)

wp5 = LocationGlobalRelative(37.8750399, 32.4752818 , height) # args = (lat, lon, alt)
waypoints.append(wp5)

wp6 = LocationGlobalRelative(37.8750257, 32.4751778, height) # args = (lat, lon, alt)
waypoints.append(wp6)

wp7 = LocationGlobalRelative(37.8750399, 32.4752818 , height) # args = (lat, lon, alt)
waypoints.append(wp7)

wp8 = LocationGlobalRelative(37.8750399, 32.4752818 , height) # args = (lat, lon, alt)
waypoints.append(wp8)

waypoints.append(homePoint)

time.sleep(1)

for i, wpp in enumerate(waypoints): # wp1-wp2-wp3-wp4-wp5-wp6-wp7-wp8-home
    distance = get_distance_metres(iha.location.global_relative_frame, wpp)
    while distance > 1:
        iha.simple_goto(wpp, 5, 5)
        print "remaining distance to waypoint {}: {}".format(i, distance)
        time.sleep(0.5)
        distance = get_distance_metres(iha.location.global_relative_frame, wpp)

print "Mode is about to LAND"
time.sleep(4) # Last hover before end of the mission

iha.mode = VehicleMode("LAND")
time.sleep(1)

iha.close()