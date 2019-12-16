import serial
import struct
import time

def beep():
    return "140 3 1 64 16 141 3"


command_map = {"passive": "128",
               "safe"   : "131",
               "full"   : "132",
               "clean"  : "135",
               "dock"   : "143",
               "beep"   : "140 3 1 64 16 141 3",
               "reset"  : "7",
               "song1"  : "141 0"
}


class Create():
    def __init__(self):
        self.connection = None
        self.sendCommandCallback = None #Function that take a "command"

    def connect(self, port):
        self.connection = serial.Serial(port, baudrate=115200, timeout=1)        

    def sendCommandASCII(self, command):
        cmd = ""
        for v in command.split():
            cmd += chr(int(v))

        self.sendCommandRaw(cmd)

    # sendCommandRaw takes a string interpreted as a byte array
    def sendCommandRaw(self, command):
        if self.connection is None:
            print "Not connected."
            raise Exception("Attempting to send command before connection")

        try:
            self.connection.write(command)

            if self.sendCommandCallback is not None:
                self.sendCommandCallback(command)

        except serial.SerialException:
            print "Lost connection"
            self.connection = None

    def queryList(self, packetIds):

        self.connection.flush()
        # while self.connection.in_waiting:
        #     self.get8Unsigned()
        req = "149 " + str(len(packetIds)) + " " + " ".join([str(id) for id in packetIds])
        self.sendCommandASCII(req)
        
        # req = struct.pack(">BB", 142, 7)
        # self.sendCommandRaw(req)
        
        self.readOpenInterfacePackets()

    def readOpenInterfacePackets(self):
        # print self.connection.in_waiting

        # time.sleep(1)
        # print self.connection.in_waiting
        print self.get8Unsigned()
        # print self.get8Unsigned()
        # while True:
        #     print self.get8Unsigned()
        # time.sleep(0.1)
        while self.connection.in_waiting:
            time.sleep(0.1)
            print self.get8Unsigned()

    # getDecodedBytes returns a n-byte value decoded using a format string.
    # Whether it blocks is based on how the connection was set up.
    def getDecodedBytes(self, n, fmt):
        
        try:
            return struct.unpack(fmt, self.connection.read(n))[0]
        except serial.SerialException:
            print "Lost connection"
            tk.tkMessageBox.showinfo('Uh-oh', "Lost connection to the robot!")
            connection = None
            return None
        except struct.error:
            print "Got unexpected data from serial port."
            return None

    # get8Unsigned returns an 8-bit unsigned value.
    def get8Unsigned(self):
        return self.getDecodedBytes(1, ">B")

    # get8Signed returns an 8-bit signed value.
    def get8Signed(self):
        return self.getDecodedBytes(1, "b")

    # get16Unsigned returns a 16-bit unsigned value.
    def get16Unsigned(self):
        return self.getDecodedBytes(2, ">H")

    # get16Signed returns a 16-bit signed value.
    def get16Signed(self):
        return self.getDecodedBytes(2, ">h")

