# import the necessary packages
# -*- coding: utf-8 -*-
from __future__ import division  # python2.x  must use this to do real division
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil  # Needed for command message definitions
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import math


"""
The set of commands demonstrated here include:
* get_vehicle_flying - 连接飞行状态的无人机
* get_vehicle_state - 获取无人机的状态参数
* print_vehicle_GPS - 显示当前无人机的GPS信息
* print_vehicle_attitude - 显示当前无人机的姿态信息
"""
def get_vehicle_flying():
    '''
    连接飞行状态的无人机
    return: vehicle'''
    vehicle = connect('/dev/ttyAMA0', baud=57600, wait_ready=True)
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
    #输出基本信息
    print(" Autopilot Firmware version: %s" % vehicle.version)
    print(" Mode: %s" % vehicle.mode.name)
    # Get Vehicle Home location - will be `None` until first set by autopilot
    while not vehicle.home_location:
        cmds = vehicle.commands
        cmds.download()
        cmds.wait_ready()
        if not vehicle.home_location:
            print(" Waiting for home location ...")
    print("\n Home location: %s" % vehicle.home_location)
    return vehicle


def get_vehicle_standing():
    '''
    连接飞行状态的无人机
    return: vehicle'''
    vehicle = connect('/dev/ttyAMA0', baud=57600, wait_ready=True)
    #输出基本信息
    print(" Autopilot Firmware version: %s" % vehicle.version)
    print(" Mode: %s" % vehicle.mode.name)
    return vehicle


def get_vehicle_state(vehicle):
    '''
    获取无人机的状态参数
    param vehicle:
    return: pitch, yaw, roll, h, lat, lon, air_speed, ground_speed'''
    hudu = 180 / np.pi
    attitude = vehicle.attitude
    pitch = attitude.pitch * hudu
    yaw = attitude.yaw * hudu
    roll = attitude.roll * hudu
    h = vehicle.location.global_relative_frame.alt
    lat = vehicle.location.global_relative_frame.lat
    lon = vehicle.location.global_relative_frame.lon
    air_speed = vehicle.airspeed
    ground_speed = vehicle.groundspeed
    return pitch, yaw, roll, h, lat, lon, air_speed, ground_speed


def print_vehicle_GPS(vehicle):
    '''
    显示当前无人机的GPS信息
    param vehicle:
    return:'''
    pitch, yaw, roll, h, lat, lon, air_speed, ground_speed = get_vehicle_state(vehicle)
    print("lat:" + str(lat) + " lon:" + str(lon)+ " h:" + str(h))


def print_vehicle_attitude(vehicle):
    '''
    显示当前无人机的姿态信息
    param vehicle:
    return:'''
    pitch, yaw, roll, h, lat, lon, air_speed, ground_speed = get_vehicle_state(vehicle)
    print("pitch:" + str(pitch) + " yaw:" + str(yaw)+ " roll:" + str(roll))


"""
The set of commands demonstrated here include:
* goto_height - 无人机垂直移动
* goto_location - 无人机水平移动
* calculate_goto_location - 计算无人机到图像中目标点的距离（北向，东向）
* calculate_vehicle_in_pix - 计算无人机在图像中的像素位置
* get_location_metres - 
* get_distance_metres - 
"""
def goto_height(vehicle, height, speed):
    '''
    无人机垂直移动
    param vehicle:
    param height: 相对于home点的高度
    param speed: 移动速度
    return: 目标高度'''
    pitch, yaw, roll, h, lat, lon, air_speed, ground_speed = get_vehicle_state(vehicle)
    target_location = LocationGlobalRelative(lat, lon, height)
    vehicle.groundspeed = speed
    vehicle.simple_goto(target_location)

    return height


def goto_location(vehicle, dNorth, dEast, speed):
    '''
    无人机水平移动
    param vehicle:
    param dNorth: 北向移动距离
    param dEast: 东向移动距离
    param speed: 移动速度
    return: 目标位置（LocationGlobalRelative）'''
    current_location = vehicle.location.global_relative_frame
    earth_radius = 6378137.0
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * current_location.lat / 180))

    # New position in decimal degrees
    new_lat = current_location.lat + (dLat * 180 / math.pi)
    new_lon = current_location.lon + (dLon * 180 / math.pi)
    target_location = LocationGlobalRelative(new_lat, new_lon, current_location.alt)
    vehicle.groundspeed = speed
    vehicle.simple_goto(target_location)

    return target_location


def calculate_goto_location(target_in_pix, vehicle_in_pix, yaw, meters_per_pix):
    '''
    计算无人机到图像中目标点的距离（北向，东向）
    param target_in_pix: 目标在图像中的位置
    param vehicle_in_pix: 无人机在图像中的位置
    param yaw: 无人机偏航
    param meters_per_pix: 每像素点对应的距离（米）
    return: 北向距离，东向距离'''
    hudu = 180 / np.pi
    d_right = (target_in_pix[0] - vehicle_in_pix[0]) * meters_per_pix
    d_up = (vehicle_in_pix[1] - target_in_pix[1]) * meters_per_pix
    d_east = d_right * np.cos(yaw / hudu) + d_up * np.sin(yaw / hudu)
    d_north = -d_right * np.sin(yaw / hudu) + d_up * np.cos(yaw / hudu)
    return d_north, d_east


def calculate_vehicle_in_pix(pitch, yaw, roll, h, meters_per_pix):
    '''
    计算无人机在图像中的像素位置，默认480*480，如果不是，需要修改函数
    param pitch: 俯仰
    param yaw: 偏航
    param roll: 滚转
    param h: 高度
    param meters_per_pix: 每像素点对应的距离（米）
    return: vehicle_in_pix'''
    hudu = 180 / np.pi
    matrix1 = [[1, 0, 0], [0, np.cos((pitch - 90) / hudu), np.sin((pitch - 90) / hudu)],
               [0, -np.sin((pitch - 90) / hudu), np.cos((pitch - 90) / hudu)]]
    matrix2 = [[np.cos(yaw / hudu), 0, -np.sin(yaw / hudu)], [0, 1, 0], [np.sin(yaw / hudu), 0, np.cos(yaw / hudu)]]
    matrix3 = [[np.cos(roll / hudu), np.sin(roll / hudu), 0], [-np.sin(roll / hudu), np.cos(roll / hudu), 0], [0, 0, 1]]
    p_in_XYZ = [0, 0, -h]
    p_in_xyz = np.dot(matrix3, np.dot(matrix2, np.dot(matrix1, p_in_XYZ)))
    p_in_pix = (int(p_in_xyz[0] / meters_per_pix) + 240, -int(p_in_xyz[2] / meters_per_pix) + 240)
    return p_in_pix


def get_location_metres(original_location, dNorth, dEast):
    earth_radius = 6378137.0
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))

    # New position in decimal degrees
    newlat = original_location.lat + (dLat * 180 / math.pi)
    newlon = original_location.lon + (dLon * 180 / math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation = LocationGlobal(newlat, newlon, original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation = LocationGlobalRelative(newlat, newlon, original_location.alt)
    else:
        raise Exception("Invalid Location object passed")
    return targetlocation


def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


def goto(vehicle, dNorth, dEast, height, speed):
    """

    """
    current_location = vehicle.location.global_relative_frame
    earth_radius = 6378137.0
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * current_location.lat / 180))
    # New position in decimal degrees
    new_lat = current_location.lat + (dLat * 180 / math.pi)
    new_lon = current_location.lon + (dLon * 180 / math.pi)
    target_location = LocationGlobalRelative(new_lat, new_lon, height)
    vehicle.groundspeed = speed
    vehicle.simple_goto(target_location)

    while vehicle.mode.name == "GUIDED":  # Stop action if we are no longer in guided mode.
        remaining_distance = get_distance_metres(vehicle.location.global_relative_frame, target_location)
        if remaining_distance <= 1 :  # Just below target, in case of undershoot.
            break
        time.sleep(2)

"""
The set of commands demonstrated here include:
* get_center - 查找大圆
* cluster_centers - 圆心聚类
* find_target_circle - 多次查找圆心
"""
def get_center(img, high_light, height, mutiplier, kernel):
    smooth = cv2.medianBlur(img, 3)
    eq = cv2.equalizeHist(smooth)
    ret, H3 = cv2.threshold(eq, high_light, 255, cv2.THRESH_BINARY_INV)

    smooth_img = cv2.medianBlur(H3, 5)
    ret, thresh = cv2.threshold(H3, 127, 255, cv2.THRESH_BINARY_INV)
    closed = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
    cv2.imwrite('/home/pi/Desktop/3/closed-' + str(high_light) + '.jpg', closed)

    # find contours
    img, contours, hier = cv2.findContours(closed, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    dist_min = 50000
    center_selected = (240, 240)
    radius_selected = 0
    # find center
    for contour in contours:
        if len(contour) > 30:
            (x, y), radius = cv2.minEnclosingCircle(contour)
            center = (int(x), int(y))
            radius = int(radius)
            perimeter = int(cv2.arcLength(contour, True))
            sur = int(cv2.contourArea(contour))
            yuan_du = int(400 * np.pi * sur / (perimeter * perimeter))
            center = (int(x * mutiplier), int(y * mutiplier))
            radius = int(radius * mutiplier)
            if yuan_du >= 75:
                radius_min = 1900 * 0.9 / height
                radius_max = 1900 * 1.1 / height
                if radius_min < radius < radius_max:
                    print('yuanDu:' + str(yuan_du) + ' radius:' + str(radius) + ' center:' + str(center))
                    dx = center[0] - int(img.shape[1] / 2)
                    dy = center[1] - int(img.shape[0] / 2)
                    dist = np.sqrt((dx * dx) + (dy * dy))
                    if dist <= dist_min:
                        dist_min = dist
                        center_selected = center
                        radius_selected = radius
    return center_selected, radius_selected


def get_center_color(img, max_color, min_color, height, mutiplier, kernel):
    ret, H1 = cv2.threshold(img, max_color, 255, cv2.THRESH_BINARY_INV)
    ret, H2 = cv2.threshold(img, min_color, 255, cv2.THRESH_BINARY)
    H3 = cv2.bitwise_and(H1, H2)

    smooth_img = cv2.medianBlur(H3, 5)
    ret, thresh = cv2.threshold(H3, 127, 255, cv2.THRESH_BINARY)
    closed = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
    cv2.imwrite('/home/pi/Desktop/3/H3-' + str(max_color) + '.jpg', H3)
    cv2.imwrite('/home/pi/Desktop/3/thresh-' + str(max_color) + '.jpg', thresh)
    cv2.imwrite('/home/pi/Desktop/3/closed-' + str(max_color) + '.jpg', closed)

    # find contours
    img, contours, hier = cv2.findContours(closed, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    dist_min = 50000
    center_selected = (240, 240)
    radius_selected = 0
    # find center
    for contour in contours:
        if len(contour) > 30:
            (x, y), radius = cv2.minEnclosingCircle(contour)
            center = (int(x), int(y))
            radius = int(radius)
            perimeter = int(cv2.arcLength(contour, True))
            sur = int(cv2.contourArea(contour))
            yuan_du = int(400 * np.pi * sur / (perimeter * perimeter))
            center = (int(x * mutiplier), int(y * mutiplier))
            radius = int(radius * mutiplier)
            if yuan_du >= 75:
                radius_min = 1900 * 0.9 / height
                radius_max = 1900 * 1.1 / height
                print(str(radius_min) + "<" + str(radius) + "<" + str(radius_max))
                if radius_min < radius < radius_max:
                    print('yuanDu:' + str(yuan_du) + ' radius:' + str(radius) + ' center:' + str(center))
                    dx = center[0] - int(img.shape[1] / 2)
                    dy = center[1] - int(img.shape[0] / 2)
                    dist = np.sqrt((dx * dx) + (dy * dy))
                    if dist <= dist_min:
                        dist_min = dist
                        center_selected = center
                        radius_selected = radius
    return center_selected, radius_selected


def cluster_centers(centers):
    clusters = np.array([[-100, -100, 0]])

    isClustered = False
    for pt in centers:
        for i in range(len(clusters)):
            dx = clusters[i][0] - pt[0]
            dy = clusters[i][1] - pt[1]
            dist = np.sqrt(dx * dx + dy * dy)
            if dist <= 20:
                isClustered = True
                clusters[i][2] = clusters[i][2] + 1
        if isClustered == False:
            clusters = np.append(clusters, pt[0])
            clusters = np.append(clusters, pt[1])
            clusters = np.append(clusters, 1)
            clusters = clusters.reshape(-1, 3)
    clusters = np.delete(clusters, 0, 0)
    return clusters


def find_target_circle(image, height):
    kernel1 = np.ones((5, 5), np.uint8)
    kernel2 = np.ones((16, 16), np.uint8)

    center = (0, 0)
    radius = 0
    width_multi = image.shape[1] / 300
    small_image = cv2.resize(image, (300, 300), interpolation=cv2.INTER_CUBIC)
    white_du = int(np.sum(small_image / 270000))
    print('The white_du is:' + str(white_du))

    hsv = cv2.cvtColor(small_image, cv2.COLOR_BGR2HSV)
    H = cv2.split(hsv)[0]
    S = cv2.split(hsv)[1]
    V = cv2.split(hsv)[2]
    centers = []
    center_selected = [240, 240]
    is_find = 0

    '''
    if white_du > 240:
        return center_selected, is_find
    elif white_du > 220:
        ranges = range(1, 6)
    elif white_du > 180:
        ranges = range(2, 7)
    elif white_du > 160:
        ranges = range(3, 8)
    else:
        ranges = range(4, 9)

    for num in ranges:
        center, radius = get_center(V, 255 - num * 10, height, width_multi, kernel1)
        if radius > 0:
            centers = np.append(centers, center)
            cv2.circle(image, (int(centers[0]), int(centers[1])), radius, (0, 255, 255), 2)
            '''


    center, radius = get_center_color(H, 135, 100, height, width_multi, kernel1)
    if radius > 0:
        centers = np.append(centers, center)
        cv2.circle(image, (int(centers[0]), int(centers[1])), radius, (0, 255, 255), 2)


    if len(centers):
        centers = centers.reshape(-1, 2)
        centers_in_weight = cluster_centers(centers)
        center_selected = (int(centers_in_weight[0][0]), int(centers_in_weight[0][1]))
        is_find = int(centers_in_weight[0][2])
    return center_selected, is_find


"""
The set of commands demonstrated here include:
* calculate_contour_attri - 计算轮廓特征
* find_contours_S - 找出S通道的轮廓
* find_contours_TF - 找出T/F的轮廓
* find_points_notice - 找出角点
* find_points_necessary - 找出关键角点
"""
def calculate_contour_attri(contour):
    perimeter = int(cv2.arcLength(contour, True))
    sur = int(cv2.contourArea(contour))
    yuan_du = int(400 * np.pi * sur / (perimeter * perimeter))
    ellipse = cv2.fitEllipse(contour)
    bian_du = int(100 * ellipse[1][0] / ellipse[1][1])
    mo = cv2.moments(contour)
    if mo['m00'] == 0:
        pos_x, pos_y = 0, 0
    else:
        pos_x, pos_y = int(mo['m10'] / mo['m00']), int(mo['m01'] / mo['m00'])
    center = (pos_x, pos_y)
    squa = np.sum(np.square(contour - center), axis=1)
    squa = np.sum(squa, axis=1)
    dists = np.sqrt(squa)
    mean_dist = int(np.sum(dists) / len(dists))
    if mean_dist == 0:
        mean_dist = 1
    dist_min = int(cv2.pointPolygonTest(contour, (pos_x, pos_y), True))
    dis_diff = int(100 * dist_min / mean_dist)
    return pos_x, pos_y, center, yuan_du, bian_du, dis_diff


def find_contours_S(image):
    kernel1 = np.uint8(np.zeros((5, 5)))

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    S = cv2.split(hsv)[1]
    ret, thresh = cv2.threshold(S, 40, 255, cv2.THRESH_BINARY_INV)
    opened = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel1)
    blured = cv2.medianBlur(opened, 5)

    img, contours, hier = cv2.findContours(blured, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    return contours


def find_contours_TF(image, contours):
    """
    查找最有可能的轮廓
    """
    status = 0
    contour_seleted = []
    copy_image = np.zeros(image.shape, np.uint8)
    for contour in contours:
        # calculate the length,sur,surLen,position,hull,color
        length = len(contour)
        if length > 50:
            pos_x, pos_y, center, yuan_du, bian_du, dis_diff = calculate_contour_attri(contour)
            cv2.drawContours(copy_image, contour, -1, (255, 255, 255), cv2.FILLED)
            is_need_color = False
            if 50 < length < 480 and 15 < pos_x < 113 and 15 < pos_y < 113 and 20 < dis_diff < 45 and 15 < yuan_du < 50 and 60 < bian_du:
                is_need_color = True
                status = 1
            if 50 < length < 480 and 15 < pos_x < 113 and 15 < pos_y < 113 and -30 < dis_diff <= 30 and 22 < yuan_du < 45 and 60 < bian_du<85:
                is_need_color = True
                status = 2
            if is_need_color:
                contour_list = list()
                contour_list.append(contour)
                cv2.drawContours(copy_image, contour_list, -1, (255, 255, 255), cv2.FILLED)
                ret, thresh2 = cv2.threshold(copy_image, 127, 255, cv2.THRESH_BINARY)
                captured_image = cv2.bitwise_and(thresh2, image)
                sum_color = np.sum(captured_image)
                sur = int(cv2.contourArea(contour))
                color = sum_color / sur / 3
                if color > 190:
                    contour_seleted = contour
                    break
    return contour_seleted, center, status


def find_points_notice(contour, center):
    """
    查找需要注意的角点
    """
    max_dist, count = 0, 0
    notice, vector = [], []
    for pt in contour:
        dist2center = np.linalg.norm(pt - center)
        if dist2center > max_dist:
            max_dist = dist2center
            vector = pt
        if dist2center != max_dist:
            count = count + 1
            if count > int(len(contour) / 11):
                notice = np.append(notice, vector)
                count = 0
                max_dist = 0
    if len(notice):
        notice = notice.reshape(-1, 2)
    return notice


def find_points_necessary(notice, center, theta):
    """
    查找必要的角点
    """
    notice_new = []
    i, n = 0, int(len(notice))
    for pt in notice:
        if i == 0:
            pt_pre = notice[n - 1]
        else:
            pt_pre = notice[i - 1]
        if i == n - 1:
            pt_next = notice[0]
        else:
            pt_next = notice[i + 1]
        i = i + 1
        dist_pre = np.linalg.norm(pt - pt_pre)
        dist_next = np.linalg.norm(pt - pt_next)
        dist2center = np.linalg.norm(pt - center)
        dist_pre2center = np.linalg.norm(pt_pre - center)
        dist_next2center = np.linalg.norm(pt_next - center)

        theta_pre = 57.3 * np.arccos((dist2center*dist2center + dist_pre2center*dist_pre2center - dist_pre*dist_pre) / (2*dist2center*dist_pre2center))
        theta_next = 57.3 * np.arccos((dist2center*dist2center + dist_next2center*dist_next2center - dist_next*dist_next) / (2*dist2center*dist_next2center))
        is_necessary = False
        if theta_pre > theta and theta_next > theta:
            is_necessary = True
        elif theta_pre > theta and theta_next <= theta and dist2center >= dist_next2center:
            is_necessary = True
        elif theta_pre <= theta and theta_next > theta and dist2center >= dist_pre2center:
            is_necessary = True
        elif theta_pre <= theta and theta_next <= theta and dist2center > dist_pre2center and dist2center > dist_next2center:
            is_necessary = True
        if is_necessary:
            notice_new = np.append(notice_new, pt)
    if len(notice_new):
        notice_new = notice_new.reshape(-1, 2)
    return notice_new

"""
The set of commands demonstrated here include:
* open_camera_csi - 打开csi摄像头
* write_title - 写标题
* write_data - 写数据
"""
def open_camera_csi():
    camera = PiCamera()
    camera.resolution = (480, 480)
    camera.framerate = 2
    rawCapture = PiRGBArray(camera, size=(480, 480))
    return camera, rawCapture


def write_title(fileObject, record_time):
    fileObject.write('Start recording data, the time is ' + record_time + '\n')
    fileObject.write('time\t' +
                     'pitch\t' +
                     'yaw\t' +
                     'roll\t' +
                     'h\t' +
                     'airS\t' +
                     'groundS\t' +
                     'vehicleP\t' +
                     'targetP\t' +
                     'goE\t' +
                     'goN\t' +
                     'lat\t' +
                     'lon\n')


def write_data(fileObject, used_time, pitch, yaw, roll, h, air_speed, ground_speed, vehicle_in_pix, target_in_pix, d_east, d_north, lat, lon):
    fileObject.write(str(int(used_time * 100) / 100) + '\t' +
                     str(int(pitch * 100) / 100) + '\t' +
                     str(int(yaw * 100) / 100) + '\t' +
                     str(int(roll * 100) / 100) + '\t' +
                     str(int(h * 100) / 100) + '\t' +
                     str(int(air_speed * 100) / 100) + '\t' +
                     str(int(ground_speed * 100) / 100) + '\t' +
                     str(vehicle_in_pix) + '\t' +
                     str(target_in_pix) + '\t' +
                     str(int(d_east * 100) / 100) + '\t' +
                     str(int(d_north * 100) / 100) + '\t' +
                     str(lat) + '\t' +
                     str(lon) + '\n')


