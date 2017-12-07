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

import threading
import socket
import sys
import time

URI = 'radio://0/80/2M'

rawTemperatureList = [0]*64

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

def _gridEye_r0_received(timestamp, data, logconf):
    global rawTemperatureList
    for i in range(8):
        rawTemperatureList[i] = data["gridEyeR0.rawDataC" + str(i)] * 0.25

def _gridEye_r1_received(timestamp, data, logconf):
    global rawTemperatureList
    for i in range(8):
        rawTemperatureList[i+8] = data["gridEyeR1.rawDataC" + str(i)] * 0.25

def _gridEye_r2_received(timestamp, data, logconf):
    global rawTemperatureList
    for i in range(8):
        rawTemperatureList[i+16] = data["gridEyeR2.rawDataC" + str(i)] * 0.25

def _gridEye_r3_received(timestamp, data, logconf):
    global rawTemperatureList
    for i in range(8):
        rawTemperatureList[i+24] = data["gridEyeR3.rawDataC" + str(i)] * 0.25

def _gridEye_r4_received(timestamp, data, logconf):
    global rawTemperatureList
    for i in range(8):
        rawTemperatureList[i+32] = data["gridEyeR4.rawDataC" + str(i)] * 0.25

def _gridEye_r5_received(timestamp, data, logconf):
    global rawTemperatureList
    for i in range(8):
        rawTemperatureList[i+40] = data["gridEyeR5.rawDataC" + str(i)] * 0.25

def _gridEye_r6_received(timestamp, data, logconf):
    global rawTemperatureList
    for i in range(8):
        rawTemperatureList[i+48] = data["gridEyeR6.rawDataC" + str(i)] * 0.25

def _gridEye_r7_received(timestamp, data, logconf):
    global rawTemperatureList
    for i in range(8):
        rawTemperatureList[i+56] = data["gridEyeR7.rawDataC" + str(i)] * 0.25

# def gridEye_callback(timestamp, data, logconf):
#     global range_last_x_f
#     global range_last_x_b
#     global range_last_y
#     global range_last_z
#     global range_last_z_d
#     global extVbat
#     global extCurr
#     range_last_x_f = data['range.xfrange']
#     range_last_x_b = data['range.xbrange']
#     range_last_y   = data['range.yrange']
#     range_last_z   = data['range.zrange']
#     range_last_z_d = data['range.zdrange']
#     vbat = data['pm.vbat']
#     state = data['pm.state']
#     print("xfrange: {}, xbrange: {}, yrange: {}, zrange: {}, zdrange: {}".format(range_last_x_f, range_last_x_b, range_last_y, range_last_z, range_last_z_d))
#     print("vbat: {}, state: {}".format(vbat, state))

def start_gridEye(scf):
    # log_conf = LogConfig(name='Position', period_in_ms=100)
    # log_conf.add_variable('range.xfrange', 'uint16_t')
    # log_conf.add_variable('range.xbrange', 'uint16_t')
    # log_conf.add_variable('range.yrange', 'uint16_t')
    # log_conf.add_variable('range.zrange', 'uint16_t')
    # log_conf.add_variable('range.zdrange', 'uint16_t')
    # log_conf.add_variable('pm.vbat', 'float')
    # log_conf.add_variable('pm.state', 'int8_t')
    # scf.cf.log.add_config(log_conf)
    # log_conf.data_received_cb.add_callback(gridEye_callback)
    # log_conf.start()

    groupIndex = ['R0', 'R1', 'R2', 'R3', 'R4', 'R5', 'R6', 'R7']
    gridEye_callback_group = {'R0':_gridEye_r0_received, 'R1':_gridEye_r1_received, \
                              'R2':_gridEye_r2_received, 'R3':_gridEye_r3_received, \
                              'R4':_gridEye_r4_received, 'R5':_gridEye_r5_received, \
                              'R6':_gridEye_r6_received, 'R7':_gridEye_r7_received}
    groupName = 'GridEye'
    variableName = 'gridEye'
    for i in range(len(groupIndex)):
        log_conf = LogConfig(name=groupName + groupIndex[i], period_in_ms=100)
        log_conf.add_variable(variableName + groupIndex[i] + '.rawDataC0', 'uint16_t')
        log_conf.add_variable(variableName + groupIndex[i] + '.rawDataC1', 'uint16_t')
        log_conf.add_variable(variableName + groupIndex[i] + '.rawDataC2', 'uint16_t')
        log_conf.add_variable(variableName + groupIndex[i] + '.rawDataC3', 'uint16_t')
        log_conf.add_variable(variableName + groupIndex[i] + '.rawDataC4', 'uint16_t')
        log_conf.add_variable(variableName + groupIndex[i] + '.rawDataC5', 'uint16_t')
        log_conf.add_variable(variableName + groupIndex[i] + '.rawDataC6', 'uint16_t')
        log_conf.add_variable(variableName + groupIndex[i] + '.rawDataC7', 'uint16_t')
        scf.cf.log.add_config(log_conf)
        log_conf.data_received_cb.add_callback(gridEye_callback_group[groupIndex[i]])
        log_conf.start()

def position_send(scf, x, y, z):
    cf = scf.cf
    cf.extpos.send_extpos(x, y, z+3.4)
    print("External position: {},{},{}".format(x, y, z))

def gridEye_sender(scf):
    def sendTemperature():
        global rawTemperatureList
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        server_address = ('localhost', 10000)
        while sock is not None:
            try:
                sensingTime = int(time.time()*1000000000)
                sock.sendto('{}, {}, {},'.format(30, rawTemperatureList, int(sensingTime/1000)).encode(), server_address)
                time.sleep(0.075)
            except Exception:
                continue

        # while zmqSocket is not None:
        #     try:
        #         msg = zmqSocket.recv_json()
        #         if msg["detect"]:
        #             position_send(scf, msg["pos"][0], msg["pos"][1], msg["pos"][2])
        #     except Exception:
        #         continue
    receive_thread = threading.Thread(target=sendTemperature, daemon=True)
    receive_thread.start()

if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(URI) as scf:
        start_gridEye(scf)
        gridEye_sender(scf)
        time.sleep(10)
        cf = scf.cf

        cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(1)
        cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(5)

        for y in range(30):
            # cf.commander.send_zdistance_setpoint(0, 0, 0, y / 25)
#            cf.commander.send_hover_setpoint(0, 0, 0, y / 25)
            time.sleep(0.1)

        for _ in range(20):
#            cf.commander.send_hover_setpoint(0, 0, 0, 1.2)
            time.sleep(0.1)

        # for _ in range(10):
        #     cf.commander.send_hover_setpoint(0, 1, 180, 0.4)
        #     time.sleep(0.1)

        for _ in range(100):
#            cf.commander.send_hover_setpoint(0, 0, 0, 1.2)
            time.sleep(0.1)

        for _ in range(100):
#            cf.commander.send_hover_setpoint(0, 0, 0, 1.2)
            time.sleep(0.1)

        for y in range(60):
#            cf.commander.send_hover_setpoint(0, 0, 0, (60 - y) / 50)
            time.sleep(0.1)

        cf.commander.send_stop_setpoint()
