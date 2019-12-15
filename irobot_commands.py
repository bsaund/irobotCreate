import serial

def beep():
    return "140 3 1 64 16 141 3"

def request(packet_ids):
    return "149 " + str(len(packet_ids)) + " " + " ".join([str(id) for id in packet_ids])


class Create():
    def __init__(self):
        self.connection = None

    def connect(self, port):
        self.connection = serial.Serial(port, baudrate=115200, timeout=1)        
