import socket
import pickle
import serial
import argparse

PORT = 8000
MSG_SIZE = 1024


class RemoteSerialServer:
    def __init__(self, address=('', PORT), max_clients=1):
        self.s = socket.socket()
        self.s.bind(address)
        self.s.listen(max_clients)
        self.client = None
        self.address = None
        self.serial_port = None
        baud_rate=115200
        timeout=1
        self.serial_port = serial.Serial(port="/dev/ttyUSB0",
                                         baudrate=baud_rate,
                                         bytesize=serial.EIGHTBITS,
                                         parity=serial.PARITY_NONE,
                                         stopbits=serial.STOPBITS_ONE,
                                         timeout=timeout,
                                         writeTimeout=timeout,
                                         xonxoff=False,
                                         rtscts=False,
                                         dsrdtr=False)

    def wait_for_connection(self):
        print("Waiting for connection")
        self.client, self.address = (self.s.accept())
        print('Got a connection from: {}'.format(self.address))

    def receive(self):
        print("Waiting for data")
        while True:
            raw_data = self.client.recv(MSG_SIZE)
            if not raw_data:
                self.wait_for_connection()
                continue

            print(data)
            action, data = pickle.loads(raw_data)
            response = self.handle_action(action, data)
            print(response)
            self.client.sendall(pickle.dumps(response))

    def handle_action(self, action, data):
        if action == "write":
            self.serial_port.write(data)
            return (True,)
        elif action == "read":
            data = self.serial_port.read(data)
            return True, data
        elif action == "clear":
            while self.serial_port.in_waiting:
                print("clearing serial")
                self.serial_port.read(self.serial_port.in_waiting)
            return (True,)
        elif action == "flush":
            self.serial_port.flushInput()
            return (True,)
        else:
            print("Unknown action: {}".format(action))


class RemoteSerialClient:
    def __init__(self, ip="192.168.1.54", port=PORT):
        address = (ip, port)
        self.s = socket.socket()
        self.s.connect(address)

    def send(self, action, data=None):
        self.s.sendall(pickle.dumps((action, data)))
        return pickle.loads(self.s.recv(MSG_SIZE))

    def write(self, data):
        response = self.send("write", data)
        if not response[0]:
            raise Exception("Error when writing")

    def read(self, size):
        response = self.send("read", size)
        if not response[0]:
            raise Exception("Error when reading")
        return response[1]

    def clear(self):
        self.send("clear")

    def flush(self):
        response = self.send("flush")
        if not response[0]:
            raise Exception("Error when flushing")

    def close(self):
        pass


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--server', action="store_true")
    parser.add_argument('--client', action="store_true")
    args = parser.parse_args()

    if args.server:
        s = RemoteSerialServer()
        s.wait_for_connection()
        s.receive()
    elif args.client:
        c = RemoteSerialClient()
        c.send()
    else:
        raise Exception("Must specify either client or server")
