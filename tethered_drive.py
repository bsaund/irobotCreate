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

# from Tkinter import *
import Tkinter as tk
import tkMessageBox
import tkSimpleDialog
import irobot_commands as ic

import struct
import sys, glob # for listing serial ports

try:
    import serial
except ImportError:
    tk.tkMessageBox.showerror('Import error', 'Please install pyserial.')
    raise

TEXTWIDTH = 40 # window width, in characters
TEXTHEIGHT = 48 # window height, in lines

VELOCITYCHANGE = 200
ROTATIONCHANGE = 300

helpText = """\
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


info_fields = ["battery",
               "something",
               "something else"]

class StatusWindow(tk.Frame):
    def __init__(self, parent):
        tk.Frame.__init__(self, parent)
        self.fields = {}
        self.values = {}

        for i in range(len(info_fields)):
            f = info_fields[i]
            self.fields[f] = tk.Label(self, text=f).grid(column=0, row=i, padx=20, stick=tk.W)
            self.values[f] = tk.Label(self, text="0", width=12, anchor="w").grid(column=1, row=i, padx=20)
        

class Console(tk.Frame):
    def __init__(self, parent):
        tk.Frame.__init__(self, parent)
        self.text = tk.Text(self, height = TEXTHEIGHT, width = TEXTWIDTH, wrap = tk.WORD)
        self.scroll = tk.Scrollbar(self, command=self.text.yview)
        self.text.configure(yscrollcommand=self.scroll.set)
        self.text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.scroll.pack(side=tk.RIGHT, fill=tk.Y)

        self.text.insert(tk.END, helpText)

        

class TetheredDriveApp(tk.Tk):
    keyPressed = {s:False for s in ["UP", "DOWN", "LEFT", "RIGHT"]}
    callbackKeyLastDriveCommand = ''

    def __init__(self):
        tk.Tk.__init__(self)
        self.title("iRobot Create 2 Tethered Drive")
        self.option_add('*tearOff', tk.FALSE)

        self.menubar = tk.Menu()
        self.configure(menu=self.menubar)

        createMenu = tk.Menu(self.menubar, tearoff=False)
        self.menubar.add_cascade(label="Create", menu=createMenu)

        createMenu.add_command(label="Connect", command=self.onConnect)
        createMenu.add_command(label="Help", command=self.onHelp)
        createMenu.add_command(label="Quit", command=self.onQuit)

        self.console = Console(self)
        self.console.pack(side=tk.LEFT, expand=True, fill=tk.BOTH)

        self.status_window = StatusWindow(self)
        self.status_window.pack(side=tk.RIGHT)

        self.bind("<Key>", self.callbackKey)
        self.bind("<KeyRelease>", self.callbackKey)

        self.robot = ic.Create()
        self.robot.sendCommandCallback = self.sendCommandCallback
        self.onLoad()


    def sendCommandCallback(self, command):
        print ' '.join([ str(ord(c)) for c in command ])
        self.console.text.insert(tk.END, ' '.join([ str(ord(c)) for c in command ]))
        self.console.text.insert(tk.END, '\n')
        self.console.text.see(tk.END)



    # A handler for keyboard events. Feel free to add more!
    def callbackKey(self, event):
        k = event.keysym.upper()
        motionChange = False

        if event.type == '2': # KeyPress; need to figure out how to get constant
            key_map = {"P"     : "passive",
                       "S"     : "safe",
                       "F"     : "full",
                       "C"     : "clean",
                       "D"     : "dock",
                       "SPACE" : "beep",
                       "R"     : "reset",
                       "1"     : "song1"
                       }

            if k in key_map:
                self.robot.sendCommandASCII(ic.command_map[key_map[k]])
            elif k in self.keyPressed:
                motionChange = True
                self.keyPressed[k] = True
            elif k == "Z":
                # self.robot.sendCommandASCII(ic.beep())
                self.robot.queryList([6, 17])
            else:
                print repr(k), "not handled"
        elif event.type == '3': # KeyRelease; need to figure out how to get constant
            # print k, "released"

            
            if k in self.keyPressed:
                motionChange = True
                self.keyPressed[k] = False
            
        if motionChange:
            velocity = VELOCITYCHANGE * (self.keyPressed["UP"] - self.keyPressed["DOWN"])
            rotation = ROTATIONCHANGE * (self.keyPressed["LEFT"] - self.keyPressed["RIGHT"])

            # compute left and right wheel velocities
            vr = velocity + (rotation/2)
            vl = velocity - (rotation/2)

            # create drive command
            cmd = struct.pack(">Bhh", 145, vr, vl)
            if cmd != self.callbackKeyLastDriveCommand:
                self.robot.sendCommandRaw(cmd)
                self.callbackKeyLastDriveCommand = cmd

    def onLoad(self):

        try:
            ports = self.getSerialPorts()
        except EnvironmentError:
            print "Failed to get serial ports"
            
        for port in ports:
            if not port.startswith("/dev/ttyUSB"):
                continue
            try:
                self.robot.connect(port)
                print "Connected to", port
            except:
                print "Failed to connect to", port

    def onConnect(self):
        if self.robot.connection is not None:
            tk.tkMessageBox.showinfo('Oops', "You're already connected!")
            return

        try:
            ports = self.getSerialPorts()
            port = tk.tkSimpleDialog.askstring('Port?', 'Enter COM port to open.\nAvailable options:\n' + '\n'.join(ports))
        except EnvironmentError:
            port = tk.tkSimpleDialog.askstring('Port?', 'Enter COM port to open.')

        if port is not None:
            print "Trying " + str(port) + "... "
            try:
                self.robot.connect(port)
                print "Connected!"
                tk.tkMessageBox.showinfo('Connected', "Connection succeeded!")
            except:
                print "Failed."
                tk.tkMessageBox.showinfo('Failed', "Sorry, couldn't connect to " + str(port))


    def onHelp(self):
        tk.tkMessageBox.showinfo('Help', helpText)

    def onQuit(self):
        if tk.tkMessageBox.askyesno('Really?', 'Are you sure you want to quit?'):
            self.destroy()

    def getSerialPorts(self):
        """Lists serial ports
        From http://stackoverflow.com/questions/12090503/listing-available-com-ports-with-python

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of available serial ports
        """
        if sys.platform.startswith('win'):
            ports = ['COM' + str(i + 1) for i in range(256)]

        elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            # this is to exclude your current terminal "/dev/tty"
            ports = glob.glob('/dev/tty[A-Za-z]*')

        elif sys.platform.startswith('darwin'):
            ports = glob.glob('/dev/tty.*')

        else:
            raise EnvironmentError('Unsupported platform')

        result = []
        for port in ports:

            try:
                s = serial.Serial(port)
                s.close()
                result.append(port)
            except (OSError, serial.SerialException):
                pass
        return result    

if __name__ == "__main__":
    app = TetheredDriveApp()
    app.mainloop()
