import serial

from src.config import SERIAL_PORT
from src.config import BAUD_RATE
from src.config import LOCALE
import time


class STM_communicator:
    def __init__(self, serial_port=SERIAL_PORT, baud_rate=BAUD_RATE):
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.connection = None

    def connect_STM(self):
        count = 10000
        while True:
            retry = False

            try:
                if count >= 10000:
                    print('Now building connection with STM')

                self.connection = serial.Serial(self.serial_port, self.baud_rate)

                if self.connection is not None:
                    print('Successfully connected with STM: ' + str(self.connection.name))
                    retry = False

            except Exception as error:
                if count >= 10000:
                    print('Connection with STM failed: ' + str(error))

                retry = True

            if not retry:
                break

            if count >= 10000:
                print('Retrying STM connection...')
                count = 0

            count += 1

    def disconnect_STM(self):
        try:
            if self.connection is not None:
                self.connection.close()
                self.connection = None

                print('Successfully closed connection with STM')

        except Exception as error:
            print('STM close connection failed: ' + str(error))

    def read_STM(self):
        try:
            message = self.connection.read(3)
            print('From STM:')
            print(message)

            if len(message) > 0:
                return message

            return None

        except Exception as error:
            print('STM read failed: ' + str(error))
            raise error

    def write_STM(self, message):
        try:
            print('To STM:')
            print(message)
            self.connection.write(message)

        except Exception as error:
            print('STM write failed: ' + str(error))
            raise error

if __name__ == '__main__':
    A = STM_communicator()
    A.connect_STM()
    message = "R2".encode()
    
    time.sleep(1)
    print("TESTING")
    A.write_STM(message)
    print("WAITING")
    time.sleep(5)
    print("RECIEVING")
    A.read_STM()
