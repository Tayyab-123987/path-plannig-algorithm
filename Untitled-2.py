from re import I
from dronekit import connect, Command, mavutil, VehicleMode
import time
from shapely.geometry import LineString, MultiPolygon, Polygon
from shapely.ops import split
import matplotlib.pyplot as plt
import matplotlib.path as mplpath
from shapely.geometry import Polygon, LineString, MultiLineString, MultiPolygon, box
from shapely.ops import linemerge, unary_union, polygonize
import numpy as np

PORT = 'udpin:0.0.0.0:14551'
TAKE_OFF_ALTITUDE = 10  # m


point_1_x=(-35.3632620)
point_1_y=149.1652373
point_1 = (point_1_x,point_1_y)
point_2_x=(-35.3588324)
point_2_y=149.1628352
point_2=(point_2_x,point_2_y)
point_3_x=(-35.3596148) 
point_3_y=149.1692911
point_3 = (point_3_x,point_3_y)
point_4_x=(-35.3630231) 
point_4_y=149.1682341
point_4 = (point_4_x,point_4_y)

def arm_and_takeoff(vehicle, altitude):
    """
    This function will arm the vehicle and fly it to mission altitude
    """
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print("Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(alt=altitude)

    while True:
        print("Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= altitude * 0.95:
            print("Reached mission altitude (" + str(altitude) + "m)")
            break
        time.sleep(1)


def main(coords):
    # Connect to the vehicle
    vehicle = connect(PORT, wait_ready=True, baud=57600)

    while not vehicle.is_armable:
        print("Waiting for vehicle to initialise...")
        time.sleep(1)

    cmds = vehicle.commands

    while not vehicle.home_location:
        cmds.download()
        cmds.wait_ready()
        if not vehicle.home_location:
            print("Waiting for home location...")

    cmds.clear()

    # Take off  command
    pts = list(coords)
    x, y = zip(*pts)
    cmds.add(
        Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0,
            0, 0, 0, 0, TAKE_OFF_ALTITUDE))
    i = 0
    while i is not len(coords) :
        
        cmds.add(
        Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0,
            0, 0, x[i], y[i], TAKE_OFF_ALTITUDE))
        i = i+1

    cmds.upload()
    print("Mission generation complete")

    print("Search mission uploaded successfully!")
    arm_and_takeoff(vehicle, TAKE_OFF_ALTITUDE)

    vehicle.mode = VehicleMode("AUTO")
    time.sleep(1310)
    
    vehicle.mode = VehicleMode("RTL")

def plot_coords(coords):
    pts = list(coords)
    x, y = zip(*pts)
    plt.plot(x, y)


def plot_polys(polys):
    for poly in polys:
        if (not getattr(poly, "exterior", None)):
            print("got line ?")
        plot_coords(poly.exterior.coords)
    for hole in poly.interiors:
        plot_coords(hole.coords)


def splitPolygon(polygon, nx, ny):
    minx, miny, maxx, maxy = polygon.bounds
    dx = (maxx - minx) / nx  # width of a small part
    dy = (maxy - miny) / ny  # height of a small part
    horizontal_splitters = [LineString(
        [(minx, miny + i*dy), (maxx, miny + i*dy)]) for i in range(ny)]
    vertical_splitters = [LineString(
        [(minx + i*dx, miny), (minx + i*dx, maxy)]) for i in range(nx)]
    splitters = horizontal_splitters + vertical_splitters
    result = polygon

    for splitter in splitters:
        result = MultiPolygon(split(result, splitter))

    return result


def center_coords(polygon, nx, ny):
    minx, miny, maxx, maxy = polygon.bounds
    dx = (maxx - minx) / nx  # width of a small part
    dy = (maxy - miny) / ny  # height of a small part
    cent_x = point_1_x
    cent_y = point_3_y
    temp1 = []
    poly_path = mplpath.Path(np.array(
        [point_1,point_2,point_3,point_4]))
    for i in range(ny):
        cent_y = cent_y - dy/2

        for j in range(nx):
            cent_x = dx/2 + cent_x
            point = (cent_x, cent_y)
            if (poly_path.contains_point(point) == True):

                temp1.append(point)
                plot_coords(temp1)
                plt.scatter(cent_x, cent_y, c="orange")
            cent_x = cent_x + dx/2
        cent_y = cent_y - dy/2
        cent_x = point_1_x
    temp1.reverse()
    return temp1


x = [-35.3632620, -35.3588324, -35.3596148, -35.3630231]
y = [149.1652373, 149.1628352, 149.1692911, 149.1682341]

poly = Polygon([point_1,point_2,point_3,point_4])
zero = splitPolygon(poly, 16, 16)
plot_polys(zero)
why = center_coords(poly, 16, 16)
plt.plot(x, y, c="lightblue", linewidth="3")
plot_coords(why)
plt.show()
main(why)