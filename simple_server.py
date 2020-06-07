import socket


class Server:
    def __init__(self, address=('', 5000), max_clients=1):
        self.s = socket.socket()
        self.s.bind(address)
        self.s.listen(max_clients)
        self.client = None
        self.address=None

    def WaitForConnection(self):
        self.client, self.address = (self.s.accept())
        print('Got a connection from: ' + str(self.client) + '.')


s = Server()
s.WaitForConnection()
