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

import csv

URI = 'radio://0/80/250K'

TAKE_OFF = 0
HOVER    = 1
LANDING  = 2
INTERUPT = 3

DESIRED_HEIGHT_CM = 140
TIME_FACTOR   = 2

mode = TAKE_OFF
last_mode = TAKE_OFF
trajectory = []
count = 0
key = ''
has_interrupt = False
has_emergency = False

vPX = 0
vPY = 0
vPZ = 0
zrange = 0
sX = 0
sY = 0
sZ = 0
zg = 0

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
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)

def change_mode(next_mode):
    global mode, count
    mode = next_mode
    count = 0
    trajectory_generator()

def check_interrupt():

    global mode, key, has_interrupt, has_emergency

    key = sys.stdin.read(1)
    if (key == 'l'):
        print("landing!!")
        change_mode(LANDING)
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
    elif (key == 'q'):
        has_emergency = True
        print('emergency!!')


def trajectory_generator():

    global trajectory, mode

    trajectory = []

    # 40 cm -> 10/25, 0.4 -> Desired_Height_CM=40, 10=Desired_Height_CM/4, 0.4=Desired_Height_CM/100
    # 80 cm -> 20/25, 0.8 -> Desired_Height_CM=80, 20=Desired_Height_CM/4, 0.8=Desired_Height_CM/100
    # 120 cm -> 30/25, 1.2 -> Desired_Height_CM=120, 30=Desired_Height_CM/4, 1.2=Desired_Height_CM/100

    if mode == TAKE_OFF:
        for y in range( (int)((DESIRED_HEIGHT_CM/4.0)*TIME_FACTOR) ):
            trajectory.append( (0,0,0, (y/ (25*TIME_FACTOR))) )

    elif mode == HOVER:
        trajectory.append( (0,0,0,DESIRED_HEIGHT_CM/100) )

    elif mode == LANDING:
        for y in range( (int)((DESIRED_HEIGHT_CM/4.0)*TIME_FACTOR) ) :
            trajectory.append( (0,0,0, ( ((DESIRED_HEIGHT_CM/4)*TIME_FACTOR)-y ) / (25*TIME_FACTOR) ) )

    return trajectory

def action_update():

    global mode, trajectory, count

    if mode == TAKE_OFF:
        if count == len(trajectory):
            mode = HOVER
            trajectory = trajectory_generator()
            count = 0
            return trajectory[count]
        else:
            action = trajectory[count]
            count += 1
            return action

    elif mode == HOVER:
        return trajectory[count]

    elif mode == LANDING:
        if count == (len(trajectory)-1):
            return trajectory[count]
        action = trajectory[count]
        count += 1
        return action

def do_action(cf, action):

    global has_interrupt, key

    if mode == TAKE_OFF or mode == LANDING:
        if has_interrupt:
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

        if has_emergency:
            f.close()
            sys.exit()

        print ("mode: ", mode)
        print ("next action: ", action[0], action[1], action[2], action[3])
        cf.commander.send_zdistance_setpoint(action[0], action[1], action[2], action[3])
        time.sleep(0.1)

    elif mode == HOVER:
        if has_interrupt:
            if key == 'j':
                action = ( action[0], (action[1]+0.2), action[2], action[3] )
            elif key == 'k':
                action = ( action[0], (action[1]-0.2), action[2], action[3] )
            elif key == 'i':
                action = ( (action[0]+0.2), action[1], action[2], action[3] )
            elif key == 'm':
                action = ( (action[0]-0.2), action[1], action[2], action[3] )
            has_interrupt = False
            key = ''

        if has_emergency:
            f.close()
            sys.exit()

        print ("mode: ", mode)
        print ("next action: ", action[0], action[1], action[2], action[3])
        cf.commander.send_hover_setpoint(action[0], action[1], action[2], action[3])
        time.sleep(0.1)

def position_callback(timestamp, data, logconf):

    global vPX, vPY, vPZ, zrange, sX, sY, sZ, zg

    sX = data['kalman.stateX']
    sY = data['kalman.stateY']
    sZ = data['kalman.stateZ']
    zg = data['kalman_states.zg']
    zrange = data['range.zrange']
    # vPX = data['kalman.varPX']
    # vPY = data['kalman.varPY']
    # vPZ = data['kalman.varPZ']

    print('sX:{}, sY:{}, sZ:{}, zg:{}, zrange:{}, vPX:{}, vPY:{}, vPZ:{})'.format(sX, sY, sZ, zg, zrange, vPX, vPY, vPZ))

def prediction_callback(timestamp, data, logconf):

    # global vPX, vPY, vPZ, zrange, sX, sY, sZ, zg

    predNX = data['kalman_pred.predNX']
    predNY = data['kalman_pred.predNY']
    measNX = data['kalman_pred.measNX']
    measNY = data['kalman_pred.measNY']

    print('predNX:{}, predNY:{}, measNX:{}, measNY:{})'.format(predNX, predNY, measNX, measNY))


def start_position_printing(scf):
    log_conf = LogConfig(name='Position', period_in_ms=100)
    # log_conf.add_variable('kalman.varPX', 'float')
    # log_conf.add_variable('kalman.varPY', 'float')
    # log_conf.add_variable('kalman.varPZ', 'float')
    log_conf.add_variable('range.zrange', 'uint16_t')
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')
    log_conf.add_variable('kalman_states.zg', 'float')

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(position_callback)
    log_conf.start()

    log_conf = LogConfig(name='Prediction', period_in_ms=100)
    log_conf.add_variable('kalman_pred.predNX', 'float')
    log_conf.add_variable('kalman_pred.predNY', 'float')
    log_conf.add_variable('kalman_pred.measNX', 'float')
    log_conf.add_variable('kalman_pred.measNY', 'float')

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(prediction_callback)
    log_conf.start()

def automatic(scf, f):

    cf = scf.cf
    trajectory = trajectory_generator()

    while True:
        check_interrupt()
        action = action_update()
        do_action(cf, action)
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

    with raw(sys.stdin):
        with nonblocking(sys.stdin):
            with SyncCrazyflie(URI) as scf:
                reset_estimator(scf)
                start_position_printing(scf)
                automatic(scf, f)

