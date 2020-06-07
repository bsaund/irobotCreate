import socket


class Client:
    def __init__(self, address=("192.168.1.54", 5000)):
        self.s = socket.socket()
        self.s.connect(address)

    def send(self):
        self.s.send('hello world')


c = Client()
