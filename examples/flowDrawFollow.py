# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2016 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.
"""
Simple example that connects to the crazyflie at `URI` and runs a figure 8
sequence. This script requires some kind of location system, it has been
tested with (and designed for) the flow deck.

Change the URI variable to your Crazyflie configuration.
"""
import logging
import time

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

import atexit

import fcntl
import sys
import os
import tty
import termios
import argparse

import csv

import numpy as np
import cv2
import sys

import threading


SIDE_LENGTH = 50
PIXEL_LENGTH = 8
MARGIN = 100
GAP = 10
RADIUS = 10
TEXT_HEIGHT = 30
TEXT_HEIGHT_GAP = 10

VIDEO_Y_PIXEL = 1080
VIDEO_X_PIXEL = 1920

# B, G, R
COLOR = {'WHITE':(255, 255, 255), 'RED':(0, 0, 255), 'ORANGE':(0, 128, 255), 'YELLOW':(0, 255, 255), 'GREEN':(0, 255, 0), 'BLUE':(255, 0, 0), 'PINK':(255, 0, 255)}

redGroup = []
orangeGroup = []
yellowGroup = []
greenGroup = []

URI = 'radio://0/80/2M'

TAKE_OFF        = 0
CALIBRATE       = 1
HOVER           = 2
LANDING         = 3
INTERRUPT       = 4
FOLLOW          = 5
DANGER          = 6
ORIENTATION     = 7
GUIDE           = 8
SEARCH_BACK     = 9
INIT            = 10
MODE = ['TAKE OFF', 'CALIBRATE', 'HOVER', 'LANDING', 'INTERRUPT', 'FOLLOW', 'DANGER', 'ORIENTATION', 'GUIDE', 'SEARCH_BACK', 'INIT']

DESIRED_HEIGHT_CM = 100
TIME_FACTOR   = 3
TIME_INTERVAL = 0.1

YAWRATE = 30 # degree/sec

mode = INIT
last_mode = INIT
trajectory = []
count = 0
key = ''
has_interrupt = False
has_emergency = False
has_done      = False

zrange = 0
range_front = 0
last_range_front = 0
xw = 4.0
yw = 4.0
xh = 0.0
yh = 0.0
zMax = 0
zMin = 0
zMaxCollection = []
z_threshold_hover_high = 4
z_threshold_hover_low  = 3
z_threshold_hover_hot  = 35
z_threshold_follow_high = 4.5
z_threshold_follow_low  = 3
pre_vx = 0.0
pre_vy = 0.0
vx = 0.0
vy = 0.0
yawrate = 0.0
gross_angle = 0.0
start_time = time.time()
gross_distance = 0.0
net_distance = 0.0
search_back_flag = False
people_count = 0
command_queue = []
vbat  = 0
vstate = 0

LEFT  = 1
RIGHT = -1
vy_guide_direction = LEFT

FRONT_LEFT  = 0
FRONT_RIGHT = 1
lastSensorDirection = FRONT_LEFT

#LED CTRL
LED_RED_ON = 1<<0
LED_RED_BLINK = 1<<1
LED_YELLOW_ON = 1<<2
LED_YELLOW_BLINK = 1<<3
LED_GREEN_ON = 1<<4
LED_GREEN_BLINK = 1<<5

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

take_off_sequence = [
    # (0   , 0   , 0,  1/25),
    # (0   , 0   , 0.3, 0),
    # (0   , 0   , 0.3, 0),
    # (0   , 0   , 0.3, 0)
]

hover_sequence = [
    # (0   , 0   , 0.3, 0),
    # (0   , 0   , 0.3, 0),
    # (0   , 0   , 0.3, 0),
    # (0   , 0   , 0.3, 0)
]

landing_sequence = [
    # (0   , 0   , 0.3, 0),
    # (0   , 0   , 0.3, 0),
    # (0   , 0   , 0.3, 0),
    # (0   , 0   , 0.3, 0)
]

def resendpara(cf):
    global command_queue
    if len(command_queue) > 0:
        accept = True
        try:
            cf.param.set_value(command_queue[0][0], command_queue[0][1])
        except Exception as e:
            print (e)
            accept = False
        if accept:
            command_queue.pop(0)


def sendpara(cf, para, message):
    global command_queue
    command_queue.append((para, message))

class raw(object):
    def __init__(self, stream):
        self.stream = stream
        self.fd = self.stream.fileno()
    def __enter__(self):
        self.original_stty = termios.tcgetattr(self.stream)
        tty.setcbreak(self.stream)
    def __exit__(self, type, value, traceback):
        termios.tcsetattr(self.stream, termios.TCSANOW, self.original_stty)

class nonblocking(object):
    def __init__(self, stream):
        self.stream = stream
        self.fd = self.stream.fileno()
    def __enter__(self):
        self.orig_fl = fcntl.fcntl(self.fd, fcntl.F_GETFL)
        fcntl.fcntl(self.fd, fcntl.F_SETFL, self.orig_fl | os.O_NONBLOCK)
    def __exit__(self, *args):
        fcntl.fcntl(self.fd, fcntl.F_SETFL, self.orig_fl)

def drawPixel(pixelNum, frame, color):
    row = int(pixelNum/8)
    col = pixelNum%8
    rotatedpixelNum = (PIXEL_LENGTH-1-col)*PIXEL_LENGTH + row
    rotatedRow = int(rotatedpixelNum/8)
    rotatedCol = rotatedpixelNum%8

    x_coordinate = VIDEO_X_PIXEL - (PIXEL_LENGTH - rotatedCol)*SIDE_LENGTH - (PIXEL_LENGTH - rotatedCol - 1)*GAP - MARGIN
    y_coordinate = VIDEO_Y_PIXEL - (PIXEL_LENGTH - rotatedRow)*SIDE_LENGTH - (PIXEL_LENGTH - rotatedRow - 1)*GAP - MARGIN
    cv2.rectangle(frame,(int(x_coordinate), int(y_coordinate)),(int(x_coordinate+SIDE_LENGTH), int(y_coordinate+SIDE_LENGTH)), color,-1)

def drawThermal(frame):
    for i in range(64):
        if i in redGroup:
            drawPixel(i, frame, COLOR['RED'])
        elif i in orangeGroup:
            drawPixel(i, frame, COLOR['ORANGE'])
        elif i in yellowGroup:
            drawPixel(i, frame, COLOR['YELLOW'])
        elif i in greenGroup:
            drawPixel(i, frame, COLOR['GREEN'])
        else:
            drawPixel(i, frame, COLOR['WHITE'])

def drawHeatCenter(frame, xw, yw, color):
    rotatedXw = PIXEL_LENGTH-1-xw
    rotatedYw = PIXEL_LENGTH-1-yw
    x_coordinate = VIDEO_X_PIXEL - (PIXEL_LENGTH - rotatedXw)*SIDE_LENGTH - (PIXEL_LENGTH - rotatedXw - 1)*GAP - MARGIN + SIDE_LENGTH/2
    y_coordinate = VIDEO_Y_PIXEL - (PIXEL_LENGTH - rotatedYw)*SIDE_LENGTH - (PIXEL_LENGTH - rotatedYw - 1)*GAP - MARGIN + SIDE_LENGTH/2
    cv2.circle(frame, (int(x_coordinate), int(y_coordinate)), RADIUS, color, -1)

def draw_text(frame):
    x_coordinate = VIDEO_X_PIXEL - (PIXEL_LENGTH - 0)*SIDE_LENGTH - (PIXEL_LENGTH - 0 - 1)*GAP - MARGIN
    y_coordinate = VIDEO_Y_PIXEL - (PIXEL_LENGTH - 0)*SIDE_LENGTH - (PIXEL_LENGTH - 0 - 1)*GAP - MARGIN

    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(frame,'zMax:'+str(zMax),(x_coordinate, y_coordinate-TEXT_HEIGHT_GAP-8*TEXT_HEIGHT), font, 1,(255,255,255),2,cv2.LINE_AA)
    cv2.putText(frame,'zMin:'+str(zMin),(x_coordinate, y_coordinate-TEXT_HEIGHT_GAP-7*TEXT_HEIGHT), font, 1,(255,255,255),2,cv2.LINE_AA)
    cv2.putText(frame,'range_front:'+str(range_front),(x_coordinate, y_coordinate-TEXT_HEIGHT_GAP-6*TEXT_HEIGHT), font, 1,(255,255,255),2,cv2.LINE_AA)
    cv2.putText(frame,'mode:'+MODE[mode],(x_coordinate, y_coordinate-TEXT_HEIGHT_GAP-5*TEXT_HEIGHT), font, 1,(255,255,255),2,cv2.LINE_AA)
    msg = 'zTh S:{}/M:{}/L:{}'.format(z_threshold_hover_low, z_threshold_hover_high, z_threshold_hover_hot)
    cv2.putText(frame,msg,(x_coordinate, y_coordinate-TEXT_HEIGHT_GAP-4*TEXT_HEIGHT), font, 1,(255,255,255),2,cv2.LINE_AA)
    msg = 'gross angle:{}'.format(round(gross_angle,2))
    cv2.putText(frame,msg,(x_coordinate, y_coordinate-TEXT_HEIGHT_GAP-3*TEXT_HEIGHT), font, 1,(255,255,255),2,cv2.LINE_AA)
    msg = 'gross_distance:{}'.format(round(gross_distance,2))
    cv2.putText(frame,msg,(x_coordinate, y_coordinate-TEXT_HEIGHT_GAP-2*TEXT_HEIGHT), font, 1,(255,255,255),2,cv2.LINE_AA)
    msg = 'vx:{} vy:{} yawrate:{}'.format(round(vx,2), round(vy,2), round(yawrate,2))
    cv2.putText(frame,msg,(x_coordinate, y_coordinate-TEXT_HEIGHT_GAP-TEXT_HEIGHT), font, 1,(255,255,255),2,cv2.LINE_AA)
    msg = 'vbat:{} vstate:{}'.format(round(vbat,2), vstate)
    cv2.putText(frame,msg,(x_coordinate, y_coordinate-TEXT_HEIGHT_GAP), font, 1,(255,255,255),2,cv2.LINE_AA)

def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold_x = 0.001
    threshold_y = 0.001
    threshold_z = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            print("{} {} {}".
                  format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold_x and (
                    max_y - min_y) < threshold_y and (
                    max_z - min_z) < threshold_z:
                break

def reset_estimator(scf):
    cf = scf.cf
    sendpara(cf, 'kalman.resetEstimation', '1')
    time.sleep(TIME_INTERVAL)
    sendpara(cf, 'kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)

def change_mode(cf, next_mode):
    global mode, count, gross_angle, start_time, gross_distance
    mode = next_mode
    count = 0
    trajectory_generator()

    if mode == TAKE_OFF:
        sendpara(cf, 'zscore.zcal', '0')
        time.sleep(TIME_INTERVAL)
        sendpara(cf, 'zscore.thre_hot', str(z_threshold_hover_hot))
        time.sleep(TIME_INTERVAL)
        sendpara(cf, 'zscore.thre_high', str(z_threshold_hover_high))
        time.sleep(TIME_INTERVAL)
        sendpara(cf, 'zscore.thre_low', str(z_threshold_hover_low))
        time.sleep(TIME_INTERVAL)
        sendpara(cf, 'ledIctr.ledctrl', str(LED_GREEN_ON))
        time.sleep(TIME_INTERVAL)

    elif mode == CALIBRATE:
        sendpara(cf, 'ledIctr.ledctrl', str(LED_GREEN_ON))
        # time.sleep(TIME_INTERVAL)

    elif mode == HOVER:
        gross_angle = 0.0
        sendpara(cf, 'zscore.thre_high', str(z_threshold_hover_high))
        # time.sleep(TIME_INTERVAL / 2.0)
        sendpara(cf, 'ledIctr.ledctrl', str(LED_GREEN_ON|LED_GREEN_BLINK))
        # sendpara(cf, 'zscore.thre_low', str(z_threshold_hover_low))
        # time.sleep(0.05)

    elif mode == FOLLOW:
        gross_distance = 0.0
        sendpara(cf, 'ledIctr.ledctrl', str(LED_GREEN_ON))

    elif mode == DANGER:
        start_time = time.time()
        sendpara(cf, 'ledIctr.ledctrl', str(LED_RED_ON|LED_RED_BLINK))

    elif mode == ORIENTATION:
        gross_distance = 0.0
        sendpara(cf, 'ledIctr.ledctrl', str(LED_RED_ON|LED_RED_BLINK))

    elif mode == GUIDE:
        gross_distance = 0.0
        start_time = time.time()
        sendpara(cf, 'ledIctr.ledctrl', str(LED_RED_ON|LED_RED_BLINK|LED_YELLOW_ON|LED_YELLOW_BLINK))

    elif mode == SEARCH_BACK:
        gross_distance = 0.0
        start_time = time.time()
        sendpara(cf, 'ledIctr.ledctrl', str(LED_YELLOW_ON|LED_YELLOW_BLINK))

    elif mode == LANDING:
        sendpara(cf, 'ledIctr.ledctrl', str(LED_GREEN_ON|LED_GREEN_BLINK))

def check_interrupt(cf):

    global mode, key, has_interrupt, has_emergency

    key = sys.stdin.read(1)
    if (key == 't'):
        print("takeoff")
        change_mode(cf, TAKE_OFF)
    elif (key == 'l'):
        print("landing!!")
        change_mode(cf, LANDING)
    elif (key == 'j'):
        has_interrupt = True
        print('left a bit')
    elif (key == 'k'):
        has_interrupt = True
        print('right a bit')
    elif (key == 'i'):
        has_interrupt = True
        print('forward a bit')
    elif (key == 'm'):
        has_interrupt = True
        print('backward a bit')
    elif (key == 'r'):
        has_interrupt = True
        print('rotate a bit')
    elif (key == 'e'):
        has_interrupt = True
        print('rotate back a bit')
    elif (key == 'q'):
        has_emergency = True
        print('emergency!!')

def handle_interrupt(action):
    global has_interrupt, key

    if mode == TAKE_OFF:
        if key == 'j':
            action = ( (action[0]-20), action[1], action[2], action[3] )
        elif key == 'k':
            action = ( (action[0]+20), action[1], action[2], action[3] )
        elif key == 'i':
            action = ( action[0], (action[1]-20), action[2], action[3] )
        elif key == 'm':
            action = ( action[0], (action[1]+20), action[2], action[3] )
        has_interrupt = False
        key = ''

    elif mode == CALIBRATE or mode == HOVER or mode == LANDING:
        if key == 'j':
            action = ( action[0], (action[1]+0.2), action[2], action[3] )
        elif key == 'k':
            action = ( action[0], (action[1]-0.2), action[2], action[3] )
        elif key == 'i':
            action = ( (action[0]+0.2), action[1], action[2], action[3] )
        elif key == 'm':
            action = ( (action[0]-0.2), action[1], action[2], action[3] )
        elif key == 'r':
            action = ( action[0], action[1], (action[2]+10), action[3] )
        elif key == 'e':
            action = ( action[0], action[1], (action[2]-10), action[3] )
        has_interrupt = False
        key = ''

    return action

def constrain(value, max_value, min_value):
    return min(max_value, max(min_value, value))

def brake(pre_value, expect_value, brake_grid):
    # example1:
    # expect_value = -0.2, pre_value = 0.2, brake_grid = 0.05
    # return 0.15
    # example2:
    # expect_value = 0.2, pre_value = -0.2, brake_grid = 0.05
    # return -0.15
    # example3:
    # expect_value = 0.1, pre_value = 0.1, brake_grid = 0.05
    # return 0.1

    diff = abs(expect_value - pre_value)
    if pre_value >= expect_value:
        increment = -min(diff, brake_grid)
    else:
        increment = min(diff, brake_grid)

    return (pre_value + increment)

def gross_angle_update(yawrate):
    global gross_angle

    gross_angle += yawrate * TIME_INTERVAL
    if gross_angle > 180:
        gross_angle -= 360
    elif gross_angle < -180:
        gross_angle += 360

def trajectory_generator():

    global trajectory, mode, pre_vx, pre_vy, vx, vy, yawrate, vy_guide_direction
    global gross_angle, gross_distance, last_range_front, net_distance, search_back_flag

    trajectory = []

    # 40 cm -> 10/25, 0.4 -> Desired_Height_CM=40, 10=Desired_Height_CM/4, 0.4=Desired_Height_CM/100
    # 80 cm -> 20/25, 0.8 -> Desired_Height_CM=80, 20=Desired_Height_CM/4, 0.8=Desired_Height_CM/100
    # 120 cm -> 30/25, 1.2 -> Desired_Height_CM=120, 30=Desired_Height_CM/4, 1.2=Desired_Height_CM/100

    if mode == TAKE_OFF:
        for y in range( (int)((DESIRED_HEIGHT_CM/4.0)*TIME_FACTOR) ):
            trajectory.append( (0,0,0, (y/ (25*TIME_FACTOR))) )

    elif mode == CALIBRATE:
        for _ in range(80):
            trajectory.append( (0,0,0,DESIRED_HEIGHT_CM/100) )

    elif mode == HOVER:
        gross_angle_update(YAWRATE)
        trajectory.append( (0,0,YAWRATE,DESIRED_HEIGHT_CM/100) )

    elif mode == FOLLOW:
        if abs(3.5-xw) > 3.5:
            vx = 0
            yawrate = 0

        else:
            # example: if yawrate_grid = 0.5 and yawrate_unit = 7.5
            # section                         yawrate
            # 0 <= abs(3.5-xw) < 0.5      ->  0
            # 0.5 <= abs(3.5-xw) < 1.0    ->  7.5
            # 1.0 <= abs(3.5-xw) < 1.5    ->  15
            # 1.5 <= abs(3.5-xw) < 2.0    ->  22.5
            # 2.5 <= abs(3.5-xw) < 3.0    ->  30
            # 3.0 <= abs(3.5-xw) < 3.5    ->  37.5

            yawrate_grid = 0.5
            yawrate_unit = 7.5 # degree/sec
            yawrate = round( int(abs(3.5-xw)/yawrate_grid ) * yawrate_unit, 2)
            yawrate = constrain(yawrate, 40, 0)
            if (3.5-xw) < 0:
                yawrate = -yawrate

            detect_limit = 1300.0
            detect_grid = detect_limit / 10.0
            follow_distance = 600.0
            vx_unit = 0.2
            if range_front < detect_limit:
                vx = round( int(abs(range_front-follow_distance)/detect_grid ) * vx_unit, 2)
                vx = constrain(vx, 0.5, 0)
                if range_front - follow_distance < 0:
                    vx = -vx
            else:
                vx = 0
            brake_grid = 0.1
            vx = brake(pre_vx, vx, brake_grid)

        gross_angle_update(yawrate)
        gross_distance += (vx * TIME_INTERVAL) * 1000 # mm
        trajectory.append( (vx,0,yawrate,DESIRED_HEIGHT_CM/100) )
        pre_vx = vx

    elif mode == DANGER:
        if abs(3.5-xh) > 3.5:
            vx = 0
            yawrate = 0

        else:
            yawrate_grid = 0.5
            yawrate_unit = 7.5 # degree/sec
            yawrate = round( int(abs(3.5-xh)/yawrate_grid ) * yawrate_unit, 2)
            yawrate = constrain(yawrate, 40, 0)
            if (3.5-xh) < 0:
                yawrate = -yawrate

            # detect_limit = 1200.0
            # detect_grid = detect_limit / 10.0
            # follow_distance = 700.0
            # vx_unit = 0.1
            # if range_front < detect_limit:
            #     vx = round( int(abs(range_front-follow_distance)/detect_grid ) * vx_unit, 2)
            #     vx = constrain(vx, 0.3, 0)
            #     if range_front - follow_distance < 0:
            #         vx = -vx
            # else:
            #     vx = 0
            # brake_grid = 0.05
            # vx = brake(pre_vx, vx, brake_grid)

        gross_angle_update(yawrate)
        trajectory.append( (0,0,yawrate,DESIRED_HEIGHT_CM/100) )
        pre_vx = 0

        if xh is not -1:
            if xw <= xh:
                vy_guide_direction = LEFT
            else:
                vy_guide_direction = RIGHT

    elif mode == ORIENTATION:
        if gross_angle < 0 and gross_angle > -180:
            yawrate = -YAWRATE
        elif gross_angle > 0 and gross_angle < 180:
            yawrate = YAWRATE

        # brake_grid = 0.05
        # vy_unit = 0.1
        # vy_grid = 0.5

        # if abs(3.5-xw) > 3.5:
        #     vy = 0
        # else:
        #     vy = round(int(abs(3.5-xw)/vy_grid) * vy_unit, 2)
        #     vy = constrain(vy, 0.2, 0)
        #     if 3.5-xw > 0:
        #         vy = -vy
        # vy = brake(pre_vy, vy, brake_grid)

        gross_angle_update(yawrate)
        trajectory.append( (0,0,yawrate,DESIRED_HEIGHT_CM/100) )
        # pre_vy = vy

    elif mode == GUIDE:
        leading_distance = 1200
        search_distance = 800
        safety_distance = 600

        vx_unit = 0.5
        brake_grid_vx = 0.05
        vy_unit = 0.2
        vy_grid = 0.5
        brake_grid_vy = 0.05
        brake_grid_search = 0.015

        if abs(3.5-xw) > 3.5:
            vy = 0
            vx = 0
            yawrate = 0

        if abs(gross_distance) < leading_distance:
            if range_front > safety_distance:
                vx = -vx_unit
            else:
                vx = (-1.5)*vx_unit
                # delta_vx = ((last_range_front - range_front)/1000.0)/TIME_INTERVAL # distance/time = speed
                # if delta_vx >=0:
                #     delta_vx = constrain(delta_vx, 0.7, 0)
                #     vx = vx + (-1)*delta_vx
                # else:
                #     vx = min(-vx_unit, vx+(-1)*delta_vx)

            vx = brake(pre_vx, vx, brake_grid_vx)

            gross_distance += (vx * TIME_INTERVAL) * 1000 # mm

            if abs(3.5-xw) > 3.5:
                vy = 0
            else:
                vy = round(int(abs(3.5-xw)/vy_grid) * vy_unit, 2)
                vy = constrain(vy, 0.5, 0)
                if 3.5-xw > 0:
                    vy = -vy
            vy = brake(pre_vy, vy, brake_grid_vy)

        else:
            vy = 0
            if range_front < search_distance:
                gross_distance = 0
            else:
                vx = brake(pre_vx, 0, brake_grid_search)
                if vx == 0:
                    search_back_flag = True

        yawrate = 0
        gross_angle_update(yawrate)
        trajectory.append( (vx,vy,yawrate,DESIRED_HEIGHT_CM/100) )
        last_range_front = range_front
        pre_vx = vx
        pre_vy = vy

    elif mode == SEARCH_BACK:
        search_distance = 800
        turnback_distance = 1500

        vx_unit = 0.4
        brake_grid_vx = 0.05
        vy_unit = 0.2
        vy_grid = 0.5
        brake_grid_vy = 0.05

        if range_front >= search_distance:
            vx = vx_unit
            vx = brake(pre_vx, vx, brake_grid_vx)

            if abs(3.5-xw) > 3.5:
                vy = 0
            else:
                vy = round(int(abs(3.5-xw)/vy_grid) * vy_unit, 2)
                vy = constrain(vy, 0.5, 0)
                if 3.5-xw > 0:
                    vy = -vy
            vy = brake(pre_vy, vy, brake_grid_vy)

            if abs(gross_distance) > turnback_distance:
                vx = 0
                vy = 0
            else:
                gross_distance += (vx * TIME_INTERVAL) * 1000 # mm

        else:
            vx = 0
            vy = 0
            gross_distance = 0
            search_back_flag = False

        yawrate = 0
        gross_angle_update(yawrate)
        trajectory.append( (vx,vy,yawrate,DESIRED_HEIGHT_CM/100) )
        pre_vx = vx
        pre_vy = vy

    elif mode == LANDING:
        for y in range( (int)((DESIRED_HEIGHT_CM/4.0)*TIME_FACTOR) ) :
            trajectory.append( (0,0,0, ( ((DESIRED_HEIGHT_CM/4)*TIME_FACTOR)-y ) / (25*TIME_FACTOR) ) )

    return trajectory

def action_update():

    global mode, trajectory, count, has_done

    if mode == INIT:
        action = [-1,-1,-1,-1]
        return action

    elif mode == TAKE_OFF or mode == CALIBRATE:
        action = trajectory[count]
        count += 1
        return action

    elif mode == HOVER or mode == FOLLOW or mode == DANGER or mode == ORIENTATION or mode == GUIDE or mode == SEARCH_BACK:
        trajectory = trajectory_generator()
        count = 0
        return trajectory[count]

    elif mode == LANDING:
        if count == (len(trajectory)-1):
            if zrange < 80:
                has_done = True
            return trajectory[count]
        action = trajectory[count]
        count += 1
        return action

def mode_update(cf):
    global mode, zMaxCollection, z_threshold_hover_high, z_threshold_hover_low, z_threshold_follow_high, z_threshold_follow_low
    global people_count

    if mode == TAKE_OFF:
        if count == len(trajectory):
            change_mode(cf, CALIBRATE)

    elif mode == CALIBRATE:
        if count == len(trajectory):
            avg = np.average(zMaxCollection)
            std = np.std(zMaxCollection)
            # peak = np.max(zMaxCollection)
            peak = int(avg+std)
            print('peak:{}'.format(peak))
            if z_threshold_hover_high < peak:
                root_factor = np.sqrt(peak/z_threshold_hover_high)
                # cube_factor = np.cbrt(peak/z_threshold_hover_high)
                z_threshold_hover_high = round(z_threshold_hover_high*root_factor, 2)
                # z_threshold_hover_low  = round(z_threshold_hover_low*cube_factor, 2)
            change_mode(cf, HOVER)

        if count > int(len(trajectory)/2.0):
            zMaxCollection.append(zMax)

    elif mode == HOVER:
        if abs(3.5-xw) < 2:
            if range_front < 1000:
                people_count += 1
            else:
                people_count = 0

            if people_count > 2:
                change_mode(cf, FOLLOW)
        else:
            people_count = 0

    elif mode == FOLLOW:
        if len(redGroup) >= 3 or zMax > 50:
            change_mode(cf, DANGER)

    elif mode == DANGER:
        if time.time() - start_time > 5:
            change_mode(cf, ORIENTATION)

    elif mode == ORIENTATION:
        if abs(abs(gross_angle) - 180) < 2:
            change_mode(cf, GUIDE)

    elif mode == GUIDE:
        if search_back_flag:
            change_mode(cf, SEARCH_BACK)
        # if net_distance >= 2000:
        #     change_mode(cf, LANDING)

    elif mode == SEARCH_BACK:
        if not search_back_flag:
            change_mode(cf, GUIDE)
        if time.time() - start_time > 10:
            change_mode(cf, LANDING)

def do_action(cf, action):

    global zrange, xw, range_front

    if has_interrupt:
        action = handle_interrupt(action)

    if has_emergency or has_done:
        f.close()
        sys.exit()

    if mode == INIT:
        time.sleep(TIME_INTERVAL)

    elif mode == TAKE_OFF:
        if count < int(len(trajectory)/3.0):
            # pass
            cf.commander.send_zdistance_setpoint(action[0], action[1], action[2], action[3])
        else:
            # pass
            cf.commander.send_hover_setpoint(action[0], action[1], action[2], action[3])
        time.sleep(TIME_INTERVAL)

    else:
        # pass
        cf.commander.send_hover_setpoint(action[0], action[1], action[2], action[3])
        time.sleep(TIME_INTERVAL)

    print ("mode: ", MODE[mode])
    print ("next action: ", action[0], action[1], action[2], action[3])


def getBitfield(nL, nH):
    n = nL | (nH << 32)
    binStr = bin(n)[2:]    # remove '0b' of binary string
    rBitStr = binStr[::-1] # reverse binary string order
    rList = []
    for bit, digit in enumerate(rBitStr):
        if digit == '1':
            rList.append(bit)
    return rList

def setYellowGroup(yellowGroupL, yellowGroupH):
    global yellowGroup
    yellowGroup = getBitfield(yellowGroupL, yellowGroupH)
    # print('yellowGroup', yellowGroup)

def setOrangeGroup(orangeGroupL, orangeGroupH):
    global orangeGroup
    orangeGroup = getBitfield(orangeGroupL, orangeGroupH)
    # print('orangeGroup', orangeGroup)

def setRedGroup(redGroupL, redGroupH):
    global redGroup
    redGroup = getBitfield(redGroupL, redGroupH)
    # print('redGroup', redGroup)

def range_front_fusion(front_left, front_right):
    global lastSensorDirection
    if lastSensorDirection == FRONT_LEFT:
        if front_left > 1800:
            if front_right <= 1800:
                lastSensorDirection = FRONT_RIGHT
                return front_right
            else:
                return front_left
        else:
            return front_left
    elif lastSensorDirection == FRONT_RIGHT:
        if front_right > 1800:
            if front_left <= 1800:
                lastSensorDirection = FRONT_LEFT
                return front_left
            else:
                return front_left
        else:
            return front_right

def position_callback(timestamp, data, logconf):
    global zrange, range_front, vbat, vstate
    zrange = data['range.zrange']
    front_left = data['range.range_front_l']
    front_right = data['range.range_front_r']
    range_front = range_front_fusion(front_left, front_right)
    vbat = data['pm.vbat']
    vstate = data['pm.state']
    print('range_front:{}, zrange:{}, front_left:{}, front_right:{}, vbat:{}, vstate:{}'.format(range_front, zrange, front_left, front_right, vbat, vstate))

def gridEyeXYZ_callback(timestamp, data, logconf):
    global xw, yw, xh, yh, zMax, zMin
    xw = data['gridEyeXYZ.xw']
    yw = data['gridEyeXYZ.yw']
    xh = data['gridEyeXYZ.xh']
    yh = data['gridEyeXYZ.yh']
    zMax = data['gridEyeXYZ.zMax'] / 100
    zMin = data['gridEyeXYZ.zMin'] / 100
    # print('xw:{}, yw:{}, xh:{}, yh:{}, zMax:{}, zMin:{}'.format(xw, yw, xh, yh, zMax, zMin))

def gridEye_callback(timestamp, data, logconf):
    setYellowGroup(data["gridEye.yellowGroupL"], data["gridEye.yellowGroupH"])
    setOrangeGroup(data["gridEye.orangeGroupL"], data["gridEye.orangeGroupH"])
    setRedGroup(data["gridEye.redGroupL"], data["gridEye.redGroupH"])

def start_logging(scf):
    log_conf = LogConfig(name='Position', period_in_ms=100)
    log_conf.add_variable('range.zrange', 'uint16_t')
    log_conf.add_variable('range.range_front_l', 'uint16_t')
    log_conf.add_variable('range.range_front_r', 'uint16_t')
    log_conf.add_variable('pm.vbat', 'float')
    log_conf.add_variable('pm.state', 'int8_t')

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(position_callback)
    log_conf.start()

    log_conf = LogConfig(name='gridEyeXYZ', period_in_ms=100)
    log_conf.add_variable('gridEyeXYZ.xw', 'float')
    log_conf.add_variable('gridEyeXYZ.yw', 'float')
    log_conf.add_variable('gridEyeXYZ.xh', 'float')
    log_conf.add_variable('gridEyeXYZ.yh', 'float')
    log_conf.add_variable('gridEyeXYZ.zMax', 'int16_t')
    log_conf.add_variable('gridEyeXYZ.zMin', 'int16_t')

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(gridEyeXYZ_callback)
    log_conf.start()

    log_conf = LogConfig(name='gridEye', period_in_ms=100)
    log_conf.add_variable("gridEye.yellowGroupH", "uint32_t")
    log_conf.add_variable("gridEye.yellowGroupL", "uint32_t")
    log_conf.add_variable("gridEye.orangeGroupH", "uint32_t")
    log_conf.add_variable("gridEye.orangeGroupL", "uint32_t")
    log_conf.add_variable("gridEye.redGroupH", "uint32_t")
    log_conf.add_variable("gridEye.redGroupL", "uint32_t")

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(gridEye_callback)
    log_conf.start()

def drawCV(cap, out):
    def draw():
        while(cap.isOpened()):
            ret, frame = cap.read()
            if ret==True:
                # frame = cv2.flip(frame,180)
                # write the flipped frame
                drawThermal(frame)
                drawHeatCenter(frame, xw, yw,  COLOR['GREEN'])
                drawHeatCenter(frame, xh, yh,  COLOR['PINK'])
                draw_text(frame)
                out.write(frame)
                cv2.imshow('frame',frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                break

        cap.release()
        out.release()
        cv2.destroyAllWindows()

    opencv_thread = threading.Thread(target=draw, daemon=True)
    opencv_thread.start()

def automatic(scf, f):

    cf = scf.cf
    # trajectory = trajectory_generator()
    # change_mode(cf, TAKE_OFF)

    while True:
        check_interrupt(cf)
        action = action_update()
        do_action(cf, action)
        mode_update(cf)
        resendpara(cf)
        # write(f)

def write(f):
    global vPX, vPY, vPZ, zrange, sX, sY, sZ, zg, key
    # data_dict = {'key':key, 'vPX':vPX, 'vPY':vPY, 'vPZ':vPZ, 'zrange':zrange, 'sX':sX, 'sY':sY, 'sZ':sZ}
    # for key, value in data_dict.items():
    #     line
    #     line.append(value)
    msg = 'key:{}, sX:{}, sY:{}, sZ:{}, zrange:{}, vPX:{}, vPY:{}, vPZ:{}\n'.format(key, sX, sY, sZ, zrange, vPX, vPY, vPZ)
    f.write(msg)

if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    # csvFile = open('data.csv', 'wb')
    # csvWriter = csv.writer(csvFile, delimiter=',')
    f = open('myfile.txt', 'w')

    # filename = sys.argv[1]
    # videoId = sys.argv[2]
    parser = argparse.ArgumentParser()
    parser.add_argument("-f", "--file", dest="filename", default="test.avi" ,help="write video to FILE", metavar="FILE")
    parser.add_argument("-v", "--video", dest="videoId", default="1", help="video id number")
    args = parser.parse_args()

    cap = cv2.VideoCapture(int(args.videoId))
    cap.set(3, VIDEO_X_PIXEL)
    cap.set(4, VIDEO_Y_PIXEL)

    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(args.filename, fourcc, 25.0, (VIDEO_X_PIXEL, VIDEO_Y_PIXEL))

    with raw(sys.stdin):
        with nonblocking(sys.stdin):
            with SyncCrazyflie(URI) as scf:
                # drawCV(cap, out)
                reset_estimator(scf)
                start_logging(scf)
                drawCV(cap, out)
                automatic(scf, f)

