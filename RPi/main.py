import os
import argparse
from RPi.src.task2.algo import Task2

from src.communicator.MultiProcessCommunication import MultiProcessCommunicator


def init():
    #os.system("sudo hciconfig hci0 piscan")
    # multiprocess_communication_process = MultiProcessCommunicator()
    # multiprocess_communication_process.start()
    task2_algo = Task2()
    # note: _allow_reconnection can block the run method!
    task2_algo.run()



if __name__ == '__main__':
    init()

