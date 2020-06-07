import socket, time
import pickle


class Client:
    def __init__(self, address=("192.168.1.54", 5000)):
        self.s = socket.socket()
        self.s.connect(address)

    def send(self):
        # self.s.sendall(pickle.dumps(("this", "is", "python")))
        self.s.send(pickle.dumps("hello world"))
        print(self.s.recv(1024))


c = Client(address=("127.0.0.1", 8000))
# c = Client()
c.send()
time.sleep(1)
c.send()