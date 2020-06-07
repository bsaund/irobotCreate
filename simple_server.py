import socket


class Server:
    def __init__(self, address=('', 5000), max_clients=1):
        self.s = socket.socket()
        self.s.bind(address)
        self.s.listen(max_clients)
        self.client = None
        self.address = None

    def WaitForConnection(self):
        print("Waiting for connection")
        self.client, self.address = (self.s.accept())
        print('Got a connection from: {}.'.format(self.address))

    def receive(self):
        print("Waiting for data")
        for i in range(10):
            data = self.s.recv(1024)
            self.client.sendall("I got your message")
            print(data)


s = Server()
s.WaitForConnection()
s.receive()
