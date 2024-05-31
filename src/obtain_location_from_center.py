#!/usr/bin/env python3
import sys
import os
import time
import math
from math import sin, asin, cos, atan2, sqrt
import decimal 
from geotiff import GeoTiff
import mgrs
from PIL import Image
from PIL import ExifTags
from parseGeoTIFF import getAltFromLatLon, binarySearchNearest, getGeoFileFromUser, getGeoFileFromString
from getTarget import *
from WGS84_SK42_Translator import Translator as converter 
from SK42_Gauss_Kruger import Projector as Projector

"""
prompt the user for options input,
       then extract data from image(s)
       and use resolveTarget() to give
       target location(s)
"""
def parseImage():
    images = []
    elevationData = None
    headless = False
    if len(sys.argv) > 2:
        headless = True

    if ("--version" in sys.argv or "-v" in sys.argv or "-V" in sys.argv or
        "V" in sys.argv or "version" in sys.argv):
        #
        sys.exit(config.version)
    elif ("--help" in sys.argv or "-h" in sys.argv or
        "-H" in sys.argv or "H" in sys.argv or "help" in sys.argv):
        #
        outstr = "usage: parseImage.py [dem.tif] [Drone-Image.JPG] [Drone-Image2.JPG] [...]\n\nparseImage.py may take a GeoTIFF DEM (.tif) as input.\n\nIf provided one or more drone image filenames after,\nparseImage.py will run in headless mode and write a file of the convention:\n[Drone-Image.JPG.ATHENA]\n\nOtherwise, The user will be prompted for one or more image filenames. \nWhen finished, target match output will be displayed for each image\n"
        sys.exit(outstr)

    # If provided arguments in command line,
    #     the first argument must be a geoTiff filename
    #     and every other argument after is a drone image filename
    if len(sys.argv) > 1:
        ext = sys.argv[1].split('.')[-1].lower()
        if ext != "tif":
            if ext in ["dt0", "dt1", "dt2", "dt3", "dt4", "dt5"]:
                print(f'FILE FORMAT ERROR: DTED format ".{ext}" not supported. Please use a GeoTIFF ".tif" file!')
            outstr = f'FATAL ERROR: got first argument: {sys.argv[1]}, expected GeoTIFF ".tif" DEM!'
            sys.exit(outstr)

        elevationData, (x0, dx, dxdy, y0, dydx, dy) = getGeoFileFromString(sys.argv[1])

        if headless:
            for imageName in sys.argv[2:]:
                images.append(imageName.strip())
    else:
        # prompt the user for a filename of a GeoTIFF,
        # extract the image's elevationData and x and y params
        elevationData, (x0, dx, dxdy, y0, dydx, dy) = getGeoFileFromUser()

    nrows, ncols = elevationData.shape
    x1 = x0 + dx * ncols
    y1 = y0 + dy * nrows

    if not headless:
        print("The shape of the elevation data is: ", elevationData.shape)
        print("The raw Elevation data is: ")
        print(elevationData)

        print(f'x0: {round(x0,4)} dx: {round(dx,9)} ncols: {round(ncols,4)} x1: {round(x1,4)}')
        print(f'y0: {round(y0,4)} dy: {round(dy,9)} nrows: {round(nrows,4)} y1: {round(y1,4)}\n\n')

    xParams = (x0, x1, dx, ncols)
    yParams = (y0, y1, dy, nrows)

    if not images: # not run in headless mode
        imageName = ""
        print("\nType \'done\' to finish input\n")
        while imageName.lower() != 'done':
            print(f'Image filenames: {images}')
            imageName = str(input("Enter a drone image filename: "))
            imageName.strip()
            if imageName.lower() != 'done':
                images.append(imageName)
    while images:
        thisImage = images.pop()
        thisImage = thisImage.strip()
        sensData = None, None, None, None, None
        target = None
        try:
            fd = open(thisImage, 'rb') #read as binary
            d = str(fd.read()) # ...but convert to string
            xmp_start = d.find('<x:xmpmeta')
            xmp_end = d.find('</x:xmpmeta')
            xmp_str = d[xmp_start:xmp_end+12]
            fd.close()
            exifData = {}
            img = Image.open(thisImage)
            exifDataRaw = img._getexif()
            for tag, value in exifDataRaw.items():
                decodedTag = ExifTags.TAGS.get(tag, tag)
                exifData[decodedTag] = value
            if xmp_start != xmp_end:
                make = exifData["Make"].upper()
                make = make.strip()
                model = exifData["Model"].upper()
                model = model.strip()
                if make[-1] == "\0":
                    make = make.rstrip("\0")
                if make == "DJI":
                    sensData = handleDJI(xmp_str)
                    if sensData is not None:
                        y, x, z, azimuth, theta = sensData
                        target = resolveTarget(y, x, z, azimuth, theta, elevationData, xParams, yParams)
                    else:
                        print(f'ERROR with {thisImage}, couldn\'t find sensor data', file=sys.stderr)
                        print(f'skipping {thisImage}', file=sys.stderr)
                        continue
                else:
                    print(f'ERROR with {thisImage}, make {make} not compatible with this program!', file=sys.stderr)
                    print(f'skipping {thisImage}', file=sys.stderr)
                    continue
            else:
                print(f'ERROR with {thisImage}, xmp data not found!', file=sys.stderr)
                print(f'skipping {thisImage}', file=sys.stderr)
                continue
        except:
            print(f'ERROR with filename {thisImage}, skipping...', file=sys.stderr)
            continue
        if target is not None:
            finalDist, tarY, tarX, tarZ, terrainAlt = target
            if headless:
                output_dir = './output'
                if not os.path.exists(output_dir):
                    os.makedirs(output_dir)
                base_filename = os.path.basename(thisImage)
                filename = os.path.join(output_dir, base_filename + ".ATHENA")
                dateTime = exifData["DateTime"]
                file_object = open(filename, 'w')
                m = mgrs.MGRS()
                targetMGRS = m.toMGRS(tarY, tarX)
                targetMGRS10m = m.toMGRS(tarY,tarX, MGRSPrecision=4)
                targetMGRS100m = m.toMGRS(tarY, tarX, MGRSPrecision=3)
                file_object.write(str(tarY) + "\n")
                file_object.write(str(tarX) + "\n")
                if tarZ is None:
                    tarZ = terrainAlt
                file_object.write(str(tarZ) + "\n")
                file_object.write(str(finalDist) + "\n")
                if dateTime is not None:
                    file_object.write(str(dateTime) + "\n")
                else:
                    file_object.write("\n")
                file_object.write(targetMGRS + "\n")
                file_object.write(targetMGRS10m + "\n")
                file_object.write(targetMGRS100m + "\n")
                targetSK42Lat = converter.WGS84_SK42_Lat(float(tarY), float(tarX), float(tarZ))
                targetSK42Lon = converter.WGS84_SK42_Long(float(tarY), float(tarX), float(tarZ))
                targetSK42Alt = float(tarZ) - converter.SK42_WGS84_Alt(targetSK42Lat, targetSK42Lon, 0.0)
                file_object.write(f'{targetSK42Lat}\n')
                file_object.write(f'{targetSK42Lon}\n')
                file_object.write(f'{targetSK42Alt}\n')
                GK_zone, targetSK42_N_GK, targetSK42_E_GK = Projector.SK42_Gauss_Kruger(targetSK42Lat, targetSK42Lon)
                file_object.write(f'{GK_zone}\n')
                file_object.write(f'{targetSK42_N_GK}\n')
                file_object.write(f'{targetSK42_E_GK}\n')
                file_object.write("# format: lat, lon, alt, dist, time, MGRS 1m, MGRS 10m, MGRS 100m, SK42 Lat, SK42 Lon, SK42 Alt., SK42 Gauss-Krüger Zone, SK42 Gauss-Krüger Northing (X), SK42 Gauss-Krüger Easting (Y),  \n")
                file_object.close()
            else:
                print(f'\n\nfilename: {thisImage}')
                dateTime = exifData["DateTime"]
                if dateTime is not None:
                    print(f'Image Date/Time: {dateTime}')
                print(f'\nApproximate range to target: {int(round(finalDist))}\n')
                if tarZ is not None:
                    print(f'Approximate EGM96 alt (constructed): {math.ceil(tarZ)}')
                else:
                    tarZ = float(terrainAlt)
                print(f'Approximate EGM96 alt (terrain): {round(terrainAlt)}\n')
                print('Target:')
                print(f'WGS84 (lat, lon): {round(tarY, 6)}, {round(tarX, 6)} EGM96 Alt: {math.ceil(tarZ)}')
                print(f'Google Maps: https://maps.google.com/?q={round(tarY,6)},{round(tarX,6)}\n')
                m = mgrs.MGRS()
                targetMGRS = m.toMGRS(tarY, tarX)
                targetMGRS10m = m.toMGRS(tarY,tarX, MGRSPrecision=4)
                targetMGRS100m = m.toMGRS(tarY, tarX, MGRSPrecision=3)
                gzdEndIndex = 2
                while(targetMGRS[gzdEndIndex].isalpha()):
                    gzdEndIndex += 1
                if os.name != 'nt':
                    print(f'NATO MGRS: {targetMGRS[0:gzdEndIndex]}\033[4m{targetMGRS[gzdEndIndex:]}\033[0;0m EGM96 Alt: \033[4m{math.ceil(tarZ)}\033[0;0m')
                else:
                    print(f'NATO MGRS: {targetMGRS} EGM96 Alt: {math.ceil(tarZ)}')
                print(f'MGRS 10m: {targetMGRS10m}')
                print(f'MGRS 100m: {targetMGRS100m}\n')
                targetSK42Lat = converter.WGS84_SK42_Lat(float(tarY), float(tarX), float(tarZ))
                targetSK42Lon = converter.WGS84_SK42_Long(float(tarY), float(tarX), float(tarZ))
                targetSK42Alt = float(tarZ) - converter.SK42_WGS84_Alt(targetSK42Lat, targetSK42Lon, 0.0)
                targetSK42Alt = int(round(targetSK42Alt))
                print('SK42 (истема координат 1942 года):')
                print(f'    Geodetic (°): {round(targetSK42Lat, 6)}, {round(targetSK42Lon, 6)} Alt: {targetSK42Alt}')
                targetSK42LatDMS, targetSK42LonDMS = decimalToDegreeMinuteSecond(targetSK42Lat, targetSK42Lon)
                print('    Geodetic (° \' "):')
                print('      '+targetSK42LatDMS)
                print('      '+targetSK42LonDMS)
                GK_zone, targetSK42_N_GK, targetSK42_E_GK = Projector.SK42_Gauss_Kruger(targetSK42Lat, targetSK42Lon)
                outstr = strFormatSK42GK(GK_zone, targetSK42_N_GK, targetSK42_E_GK, targetSK42Alt)
                print(outstr)

"""takes a xmp metadata string from a drone image of type "DJI Meta Data",
returns tuple (y, x, z, azimuth, theta)

Parameters
----------
xmp_str : String
    a string containing the contents of XMP metadata of "DJI Meta Data" format
    may contain errant newline sequences, preventing parsing as true XML
elements : String[]
    optionally passed in to override XMP tags to use for search
    e.x.: with Autel drones, replace "drone-dji:GpsLatitude=" with "drone:GpsLatitude", etc.
"""
def handleDJI( xmp_str, elements=None):
    # default, unless overridden
    if elements == None:
        elements = ["drone-dji:AbsoluteAltitude=",
                    "drone-dji:GpsLatitude=",
                    "drone-dji:GpsLongitude=",
                    "drone-dji:GimbalRollDegree=", #should always be 0
                    "drone-dji:GimbalYawDegree=",
                    "drone-dji:GimbalPitchDegree=",
                    "drone-dji:FlightRollDegree=",
                    "drone-dji:FlightYawDegree=",
                    "drone-dji:FlightPitchDegree="]
    dict = xmp_parse( xmp_str, elements)
    if dict is None:
        return None
    try:
        y = float(dict[[e for e in elements if "GpsLatitude" in e][0]])
    except ValueError:
        print("Value Error")
        return None
    except TypeError:
        print(str(dict[[e for e in elements if "GpsLatitude" in e][0]]))
        print("Type Error")
        return None
    typoAgnostic = [e for e in elements if "GpsLong" in e]
    for e in typoAgnostic:
        if dict[e] == None:
            typoAgnostic.remove(e)
    if len(typoAgnostic) == 1:
        typoAgnostic = typoAgnostic[0]
    else:
        print("Typo Agnostic")
        return None
    x = float(dict[typoAgnostic])

    z = float(dict[[e for e in elements if "AbsoluteAltitude" in e][0]])

    azimuth = float(dict[[e for e in elements if "GimbalYawDegree" in e][0]])

    theta = abs(float(dict[[e for e in elements if "GimbalPitchDegree" in e][0]]))

    if azimuth == 0.0 and theta == 0.0:
        print(f'ERROR: camera orientation invalid. Your modle drone may be incompatible with this software')
        return None

    if y is None or x is None or z is None or azimuth is None or theta is None:
        return None
    else:
        return (y, x, z, azimuth, theta)

"""takes a xmp metadata string and a list of keys
return a dictionary of key, value pairs
       ...or None if xmp_str is empty

Parameters
----------
xmp_str: String
    a string containing the contents of XMP metadata
elements: String[]
    a list of strings, each string is a key for which to search XMP data
    for its corresponding value
"""
def xmp_parse ( xmp_str, elements ):
    dict = {}
    if len(xmp_str.strip()) > 0:
        for element in elements:
            if xmp_str.find(element) == -1:
                dict[element] = None
            else:
                value = xmp_str[xmp_str.find(element) + len(element) : xmp_str.find(element) + len(element) + 10]
                value = value.split('\"',3)[1]
                dict[element] = value
    else:
        return None

    return dict

"""takes a python dictionary generated from EXIF data
return a tuple (y, x, z) of latitude, longitude, and altitude
in decimal form

Parameters
----------
exifData: dict {key : value}
    a Python dictionary object containing key : value pairs of EXIF tags and their
    corresponding values

"""
def exifGetYXZ(exifData):
    GPSInfo = exifData['GPSInfo']
    latDir = GPSInfo[1].strip().upper()
    latDeg = GPSInfo[2][0]
    latMin = GPSInfo[2][1]
    latSec = GPSInfo[2][2]
    y = latDeg
    y += (latMin / 60.0)
    y += (latSec / 3600.0)
    if latDir == "S":
        y = y * -1.0
    lonDir = GPSInfo[3].strip().upper()
    lonDeg = GPSInfo[4][0]
    lonMin = GPSInfo[4][1]
    lonSec = GPSInfo[4][2]
    x = lonDeg
    x += (lonMin / 60.0)
    x += (lonSec / 3600.0)
    if lonDir == "W":
        x = x * -1.0
    altDir = GPSInfo[5] 
    z = GPSInfo[6]
    if altDir == 1:
        z *= -1.0
    try:
        y = float(y)
        x = float(x)
        z = float(z)
    except ValueError:
        print("ERROR: failed to extract GPS data from EXIF values", file=sys.stderr)
        return None
    except TypeError:
        print("ERROR: failed to extract GPS data from EXIF values", file=sys.stderr)
        return None

    return (y, x, z)

"""takes a decimal +/- Lat and Lon and returns a tuple of two strings containing Degrees Minutes Seconds each

Note: this funtion will work with Geodetic coords of any ellipsoid

Fn from Glen Bambrick: glenbambrick.com/2015/06/24/dd-to-dms/

Parameters
----------
Lat: float
    A latitude, positive or negative, in degrees
Lon: float
    A longitude, positive or negative, in degrees
"""
def decimalToDegreeMinuteSecond(Lat, Lon):
    split_degx = math.modf(Lon)
    degrees_x = int(split_degx[1])
    minutes_x = abs(int(math.modf(split_degx[0] * 60)[1]))
    seconds_x = abs(round(math.modf(split_degx[0] * 60)[0] * 60,2))
    split_degy = math.modf(Lat)
    degrees_y = int(split_degy[1])
    minutes_y = abs(int(math.modf(split_degy[0] * 60)[1]))
    seconds_y = abs(round(math.modf(split_degy[0] * 60)[0] * 60,2))

    if degrees_x < 0:
        EorW = "W"
    else:
        EorW = "E"

    if degrees_y < 0:
        NorS = "S"
    else:
        NorS = "N"

    latDMS = str(abs(degrees_y)) + "° " + str(minutes_y) + "' " + str(seconds_y) + "\" " + NorS
    lonDMS = str(abs(degrees_x)) + "° " + str(minutes_x) + "' " + str(seconds_x) + "\" " + EorW

    return (latDMS, lonDMS)

if __name__ == "__main__":
    parseImage()