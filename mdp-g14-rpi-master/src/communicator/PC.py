import socket

from src.config import WIFI_IP
from src.config import WIFI_PORT
from src.config import LOCALE
from src.Logger import Logger
from src.communicator.utils import pcMsgParser

log = Logger()

'''
PC will need an accompanying (reference available in playgrounds pc_client.py
PC.connect() will wait for PC to connect before proceeding

pc = PC()
pc.connect()
while True:
    msg = pc.read()
    print(msg)
'''

'''
Current problem: if the client side dies, the server side does not know to re-connect it again
Also i do not understand the shortforms for the msg headers
'''

class PC:
    def __init__(self, host=WIFI_IP, port=WIFI_PORT):
        self.host = host
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_sock = None
        self.address = None

    def connect(self):
        log.info('Establishing connection with PC')
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.socket.bind((self.host, self.port))
        except Exception as error:
            log.error('Connection with PC failed: ' + str(error))

        self.socket.listen(3)
        self.client_sock, self.address = self.socket.accept()
        log.info('Successfully connected with PC: ' + str(self.address))

    def disconnect(self):
        try:
            self.socket.close()
        except Exception as error:
            log.error("PC disconnect failed: " + str(error))

    def read(self):
        try:
            msg = self.client_sock.recv(2048).decode().strip()
            if len(msg) > 0:
                return pcMsgParser(msg)
            return None
        except Exception as error:
            log.error('PC read failed: ' + str(error))

    def write(self, msg):
        try:
            self.client_sock.sendto(bytes(msg + '\n', LOCALE), self.address)
            # log.info('Successfully wrote message to PC')
        except Exception as error:
            log.error('PC write failed: ' + str(error))
