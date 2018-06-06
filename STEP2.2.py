# import the necessary packages
from __future__ import division # python2.x  must use this to do real division
import time
import cv2
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import mydrone as md


# vehicle = md.get_vehicle_flying()
vehicle = md.get_vehicle_standing()
# initialize the camera and grab a reference to the raw camera capture
camera, rawCapture = md.open_camera_csi()
# allow the camera to warmup
time.sleep(2)
# initial time
time_start = time.time()
order_time = time_start

outputFile = '/home/pi/Desktop/3/data.txt'
with open(outputFile, 'a') as fileObject:
    record_time = str(time.strftime('%H-%M-%S', time.localtime(time.time())))
    md.write_title(fileObject, record_time)
    step = 0
    # capture frames from the camera
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        # grab the raw NumPy array representing the image, then initialize the timestamp
        # and occupied/unoccupied text
        image = frame.array
        copy_image = image.copy()
        # STEP0 initial
        is_find = 0
        pitch, yaw, roll, h, lat, lon, air_speed, ground_speed = md.get_vehicle_state(vehicle)
        pitch = pitch # do some fix
        roll = roll # do some fix
        meters_per_pix = 0.8447 * h / 480
        meters_per_pix = meters_per_pix / 3.5
        true_location = LocationGlobalRelative(28.2285566, 112.9943093, h)
        
        heights = np.array([9, 5, 20, 15, 12, 6])
        # STEP1.0 waiting for task active
        if step == 0:
            if h <= heights[0]:
                print("Waiting orders at height: " + str(h) + "m")
                time.sleep(1)
            else:
                step = 1
                print("Task begins.")
                vehicle.armed = True
                while not vehicle.armed:
                    print(" Waiting for arming...")
                    time.sleep(1)
        # STEP1.1 loft to 30m
        elif step == 1:
            md.goto(vehicle, 0, 0, heights[1], 1)
            print("Reach height " + str(heights[1]) + "m")
            #step = 2  # loft phrase end
            time.sleep(2)
        # STEP1.2 detect big circle at 30m, then down to 20m
        elif step == 2:
            vehicle_in_pix = md.calculate_vehicle_in_pix(pitch, yaw, roll, h, meters_per_pix)
            target_in_pix, is_find = md.find_target_circle(image, h)
            if is_find==0:
                print("Can not find circles.")
            else:
                print("Find circles, go to " + str(heights[2]) + "m.")
                d_north, d_east = md.calculate_goto_location(target_in_pix, vehicle_in_pix, yaw, meters_per_pix)
                md.goto(vehicle, d_north, d_east, heights[2], 1)
                print("Reach height " + str(heights[2]) + "m.")
                step = 3
                time.sleep(2)
        # STEP1.3 detect big circle at 20m, then down to 15m
        elif step == 3:
            vehicle_in_pix = md.calculate_vehicle_in_pix(pitch, yaw, roll, h, meters_per_pix)
            target_in_pix, is_find = md.find_target_circle(image, h)
            if is_find==0:
                print("Can not find circles.")
            else:
                print("Find circles, go to " + str(heights[3]) + "m.")
                d_north, d_east = md.calculate_goto_location(target_in_pix, vehicle_in_pix, yaw, meters_per_pix)
                md.goto(vehicle, d_north, d_east, heights[3], 1)
                print("Reach height " + str(heights[3]) + "m.")
                step = 4
                time.sleep(2)
        # STEP1.4 detect big circle at 15m, then down to 12m
        elif step == 4:
            vehicle_in_pix = md.calculate_vehicle_in_pix(pitch, yaw, roll, h, meters_per_pix)
            target_in_pix, is_find = md.find_target_circle(image, h)
            if is_find==0:
                print("Can not find circles.")
            else:
                print("Find circles, go to " + str(heights[4]) + "m.")
                d_north, d_east = md.calculate_goto_location(target_in_pix, vehicle_in_pix, yaw, meters_per_pix)
                md.goto(vehicle, d_north, d_east, heights[4], 1)
                print("Reach height " + str(heights[4]) + "m.")
                step = 5
                time.sleep(2)
        # STEP1.5 detect small circle at 12m, then down to 6m
        elif step == 5:
            current_location = vehicle.location.global_relative_frame
            locate_error = md.get_distance_metres(current_location, true_location)
            print(" locate error:" + str(locate_error))
            vehicle_in_pix = md.calculate_vehicle_in_pix(pitch, yaw, roll, h, meters_per_pix)
            target_in_pix, is_find = md.find_target_circle(image, h)
            if is_find==0:
                print("Can not find circles.")
            else:
                print("Find circles, distinguishing T/F.")
                small_image = image[(target_in_pix[1] - 64):(target_in_pix[1] + 64), (target_in_pix[0] - 64):(target_in_pix[0] + 64)]
                contours = md.find_contours_S(small_image)
                if len(contours):
                    contour, contour_center, status = md.find_contours_TF(small_image, contours)
                    if len(contour) > 400:
                        # find all points need to notice
                        notice_points = md.find_points_notice(contour, contour_center)
                        necessary_points = md.find_points_necessary(notice_points, contour_center, 45)
                        if len(necessary_points) == 3:
                            TCount = TCount + 1
                        elif len(necessary_points) == 4:
                            FCount = FCount + 1
                        else:
                            NCount = NCount + 1
            if TCount >= 3:
                step = 6
            elif FCount >= 3:
                step = 7
        elif step == 6:
            print("It is T, going to bomb.")
            break
        elif step == 7:
            print("It is F, RTL.")
            break
        # quit process
        if vehicle.mode.name != "GUIDED":
            break
        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)

