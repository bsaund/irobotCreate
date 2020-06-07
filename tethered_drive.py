#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 27 May 2015

###########################################################################
# Copyright (c) 2015 iRobot Corporation
# http://www.irobot.com/
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
#
#   Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in
#   the documentation and/or other materials provided with the
#   distribution.
#
#   Neither the name of iRobot Corporation nor the names
#   of its contributors may be used to endorse or promote products
#   derived from this software without specific prior written
#   permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
###########################################################################
from __future__ import print_function

import Tkinter as tk
import tkMessageBox
import tkSimpleDialog
from irobot.robots import create2
from irobotCreate.createlvl2 import Bradbot
from irobot.openinterface.constants import MODES

import struct
import sys, glob  # for listing serial ports
import label_mappings as lm

try:
    import serial
except ImportError:
    tk.tkMessageBox.showerror('Import error', 'Please install pyserial.')
    raise

TEXT_WIDTH = 40  # window width, in characters
TEXT_HEIGHT = 48  # window height, in lines

VELOCITY_CHANGE = 200
ROTATION_CHANGE = 300

helpText = """
Supported Keys:
P\tPassive
S\tSafe
F\tFull
C\tClean
D\tDock
R\tReset
Space\tBeep
Arrows\tMotion

If nothing happens after you connect, try pressing 'P' and then 'S' to get into safe mode.
"""

info_fields = ["Mode",
               "bump left",
               "bump right",
               "light bump left",
               "light bump front left",
               "light bump center left",
               "light bump center right",
               "light bump front right",
               "light bump right",
               "cliff left",
               "cliff front left",
               "cliff front right",
               "cliff right",
               "charging state",
               "battery charge",
               "encoder",
               "status",
               "pos"]


class StatusWindow(tk.Frame):
    def __init__(self, parent):
        tk.Frame.__init__(self, parent)
        self.fields = {}
        self.values = {}

        for i in range(len(info_fields)):
            f = info_fields[i]
            self.fields[f] = tk.Label(self, text=f)
            self.fields[f].grid(column=0, row=i, padx=20, stick=tk.W)
            self.values[f] = tk.Label(self, text="0", width=20, anchor="w")
            self.values[f].grid(column=1, row=i, padx=20)
        self.i = 0

    def set_label(self, name, text):
        self.values[name].configure(text=text)

    def refresh_labels(self, robot):
        try:
            self._refresh_labels(robot)
        except KeyError as e:
            print("Key error. Probably bad serial data")
            print("Error message: {}".format(e.message))

    def _refresh_labels(self, robot):
        all_sensors = robot.irobot_data

        self.set_label("Mode", lm.modes[all_sensors.oi_mode])
        bumps = all_sensors.bumps_and_wheel_drops
        self.set_label("bump left", bumps.bump_left)
        self.set_label("bump right", bumps.bump_right)
        lt_bump = all_sensors.light_bumper
        self.set_label("light bump left", lt_bump.left)
        self.set_label("light bump front left", lt_bump.front_left)
        self.set_label("light bump center left", lt_bump.center_left)
        self.set_label("light bump center right", lt_bump.center_right)
        self.set_label("light bump front right", lt_bump.front_right)
        self.set_label("light bump right", lt_bump.right)

        self.set_label("cliff left", all_sensors.cliff_left_sensor)
        self.set_label("cliff front left", all_sensors.cliff_front_left_sensor)
        self.set_label("cliff front right", all_sensors.cliff_front_right_sensor)
        self.set_label("cliff right", all_sensors.cliff_right_sensor)
        self.set_label("charging state", lm.battery[all_sensors.charging_state])

        self.set_label("battery charge", "%i / %i (mAh)" % (all_sensors.battery_charge, all_sensors.battery_capacity))

        self.set_label("encoder", "%i, %i" % (all_sensors.left_encoder_counts, all_sensors.right_encoder_counts))
        self.set_label("status", all_sensors.stasis.toggling)
        self.set_label("pos", "%.2f, %.2f, %.1f" % (robot.pos.x, robot.pos.y, robot.pos.theta))


class Console(tk.Frame):
    def __init__(self, parent):
        tk.Frame.__init__(self, parent)
        self.text = tk.Text(self, height=TEXT_HEIGHT, width=TEXT_WIDTH, wrap=tk.WORD)
        self.scroll = tk.Scrollbar(self, command=self.text.yview)
        self.text.configure(yscrollcommand=self.scroll.set)
        self.text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.scroll.pack(side=tk.RIGHT, fill=tk.Y)

        self.text.insert(tk.END, helpText)


class TetheredDriveApp(tk.Tk):
    keyPressed = {s: False for s in ["UP", "DOWN", "LEFT", "RIGHT"]}
    callbackKeyLastDriveCommand = ''

    def __init__(self):
        tk.Tk.__init__(self)
        self.title("iRobot Create 2 Tethered Drive")
        self.option_add('*tearOff', tk.FALSE)

        self.menubar = tk.Menu()
        self.configure(menu=self.menubar)

        createMenu = tk.Menu(self.menubar, tearoff=False)
        self.menubar.add_cascade(label="Create", menu=createMenu)

        # createMenu.add_command(label="Connect", command=self.onConnect)
        createMenu.add_command(label="Help", command=self.onHelp)
        createMenu.add_command(label="Quit", command=self.onQuit)

        self.console = Console(self)
        self.console.pack(side=tk.LEFT, expand=True, fill=tk.BOTH)

        self.status_window = StatusWindow(self)
        self.status_window.pack(side=tk.RIGHT)

        self.bind("<Key>", self.callbackKey)
        self.bind("<KeyRelease>", self.callbackKey)

        self.robot = Bradbot("/dev/ttyUSB0", remote=True)
        self.main_loop()

    def main_loop(self):
        self.robot.control_loop()
        self.status_window.refresh_labels(self.robot)
        self.after(100, self.main_loop)

    def sendCommandCallback(self, command):
        print(' '.join([str(ord(c)) for c in command]))
        self.console.text.insert(tk.END, ' '.join([str(ord(c)) for c in command]))
        self.console.text.insert(tk.END, '\n')
        self.console.text.see(tk.END)

    def callbackKey(self, event):
        k = event.keysym.upper()
        motion_change = False

        if event.type == '2':  # KeyPress; need to figure out how to get constant
            mode_map = {"P": MODES.PASSIVE,
                        "S": MODES.SAFE,
                        "F": MODES.FULL}
            action_map = {"C": "clean",
                          "D": self.robot.seek_dock,
                          "SPACE": lambda: self.robot.play_song(3),
                          "R": lambda: self.robot.go_to(0.3, 0),
                          "1": self.robot.set_katie_song
                          }

            if k in mode_map:
                self.robot.oi_mode = mode_map[k]
            elif k in self.keyPressed:
                motion_change = True
                self.keyPressed[k] = True
            elif k in action_map:
                action_map[k]()
            elif k == "Z":
                # self.robot.sendCommandASCII(ic.beep())
                self.robot.queryList([6, 17])
            else:
                print(repr(k), "not handled")
        elif event.type == '3':  # KeyRelease; need to figure out how to get constant
            # print k, "released"

            if k in self.keyPressed:
                motion_change = True
                self.keyPressed[k] = False

        if motion_change:
            velocity = VELOCITY_CHANGE * (self.keyPressed["UP"] - self.keyPressed["DOWN"])
            rotation = ROTATION_CHANGE * (self.keyPressed["LEFT"] - self.keyPressed["RIGHT"])

            # compute left and right wheel velocities
            vr = velocity + (rotation / 2)
            vl = velocity - (rotation / 2)

            self.robot.drive_direct(vr, vl)

            # # create drive command
            # cmd = struct.pack(">Bhh", 145, vr, vl)
            # if cmd != self.callbackKeyLastDriveCommand:
            #     self.robot.sendCommandRaw(cmd)
            #     self.callbackKeyLastDriveCommand = cmd

    # def onLoad(self):

    #     try:
    #         ports = self.getSerialPorts()
    #     except EnvironmentError:
    #         print "Failed to get serial ports"

    #     for port in ports:
    #         if not port.startswith("/dev/ttyUSB"):
    #             continue
    #         try:
    #             self.robot.connect(port)
    #             print "Connected to", port
    #         except:
    #             print "Failed to connect to", port

    # def onConnect(self):
    #     if self.robot.connection is not None:
    #         tk.tkMessageBox.showinfo('Oops', "You're already connected!")
    #         return

    #     try:
    #         ports = self.getSerialPorts()
    #         port = tk.tkSimpleDialog.askstring('Port?', 'Enter COM port to open.\nAvailable options:\n' + '\n'.join(ports))
    #     except EnvironmentError:
    #         port = tk.tkSimpleDialog.askstring('Port?', 'Enter COM port to open.')

    #     if port is not None:
    #         print "Trying " + str(port) + "... "
    #         try:
    #             self.robot.connect(port)
    #             print "Connected!"
    #             tk.tkMessageBox.showinfo('Connected', "Connection succeeded!")
    #         except:
    #             print "Failed."
    #             tk.tkMessageBox.showinfo('Failed', "Sorry, couldn't connect to " + str(port))

    def onHelp(self):
        tk.tkMessageBox.showinfo('Help', helpText)

    def onQuit(self):
        if tk.tkMessageBox.askyesno('Really?', 'Are you sure you want to quit?'):
            self.destroy()


if __name__ == "__main__":
    app = TetheredDriveApp()
    app.mainloop()
