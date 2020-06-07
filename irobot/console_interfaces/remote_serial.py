import socket
import pickle
import serial
import argparse


class RemoteSerialServer:
    def __init__(self, address=('', 5000), max_clients=1):
        self.s = socket.socket()
        self.s.bind(address)
        self.s.listen(max_clients)
        self.client = None
        self.address = None

    def WaitForConnection(self):
        print("Waiting for connection")
        self.client, self.address = (self.s.accept())
        print('Got a connection from: ' + str(self.client) + '.')

    def receive(self):
        print("Waiting for data")
        for i in range(10):
            data = self.s.recv(1024)
            if not data:
                continue
            data = pickle.loads(data)
            self.client.sendall("I got your message")
            print(data)


class RemoteSerialClient:
    def __init__(self, address=("192.168.1.54", 5000)):
        self.s = socket.socket()
        self.s.connect(address)

    def send(self):
        self.s.sendall(pickle.dumps(("this", "is", "python")))
        print(self.s.recv(1024))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--server', action="store_true")
    parser.add_argument('--client', action="store_true")
    args = parser.parse_args()

    if args.server:
        s = RemoteSerialServer()
        s.WaitForConnection()
        s.receive()
    elif args.client:
        c = RemoteSerialClient()
        c.send()
    else:
        raise Exception("Must specify either client or server")
