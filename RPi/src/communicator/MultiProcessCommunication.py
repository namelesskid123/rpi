import sys
sys.path.append('/home/raspberry/.virtualenvs/cv/lib/python3.9/site-packages')

from src.communicator.Android_com import Android_communicator
from src.communicator.Arduino_com import Arduino_communicator
from src.communicator.Algorithm_com import Algorithm_communicator
from src.config import STOPPING_IMAGE, IMAGE_WIDTH, IMAGE_HEIGHT, IMAGE_FORMAT
from src.ultrasonic import *
from src.protocols import *
from src.task2 import Task2, constants

from PIL import Image
import numpy as np
import imagezmq
from picamera import PiCamera
from picamera.array import PiRGBArray

import time

import collections
from multiprocessing import Process, Value
from multiprocessing.managers import BaseManager

img_count = 0


class DequeProxy(object):
    def __init__(self, *args):
        self.deque = collections.deque(*args)

    def __len__(self):
        return self.deque.__len__()

    def append(self, x):
        self.deque.append(x)

    def appendleft(self, x):
        self.deque.appendleft(x)

    def popleft(self):
        return self.deque.popleft()

    def empty(self):
        if self.deque:
            return False
        else:
            return True


class DequeManager(BaseManager):
    pass


DequeManager.register('DequeProxy', DequeProxy,
                    exposed=['__len__', 'append', 'appendleft', 'popleft', 'empty'])


class MultiProcessCommunicator:
    """
    This class will carry out multi-processing communication process between Algorithm, Android and Arduino.
    """

    def __init__(self, image_processing_server_url: str = 'tcp://192.168.14.11:5555'):
        """
        Start the multiprocessing process and set up all the relevant variables

        Upon starting, RPi will connect to individual devices in the following order
        - Algorithm
        - Arduino
        - Android

        The multiprocessing queue will also be instantiated
        """
        print('Starting Multiprocessing Communication')

        self.arduino = Arduino_communicator()  # handles connection to Arduino
        self.android = Android_communicator()  # handles connection to Android
        self.task_2 = Task2(self._add_STM_command_to_queue, self._take_pic, self._update_img_deque)

        self.manager = DequeManager()
        self.manager.start()

        # messages from Arduino, Algorithm and Android are placed in this queue before being read
        self.message_deque = self.manager.DequeProxy()
        self.to_android_message_deque = self.manager.DequeProxy()

        # self.read_arduino_process = Process(target=self._read_arduino)
        self.read_STM_process = Process(target=self._read_STM)
        # self.read_algorithm_process = Process(target=self._read_algorithm)
        self.read_android_process = Process(target=self._read_android)

        self.write_process = Process(target=self._write_target)
        self.write_android_process = Process(target=self._write_android)

        self.cur_img_result = None

        # the current action / status of the robot
        self.status = Status.IDLE  # robot starts off being idle

        self.dropped_connection = Value('i', 0)  # 0 - arduino, 1 - algorithm

        # for image recognition
        self.image_process = None
        self.image_deque = None
        self.total_img_count = 0

        if image_processing_server_url is not None:
            print('------Start Exploration with image recognition-----')
            self.image_process = Process(target=self._process_pic)
            # pictures taken using the PiCamera are placed in this queue
            self.image_deque = self.manager.DequeProxy()

            self.image_processing_server_url = image_processing_server_url
            self.image_count = Value('i', 0)
            print("self.image_deque:", self.image_deque)

    def start(self):
        try:
            # self.algorithm.connect_algo()
            self.arduino.connect_arduino()
            self.android.connect_android()

            print('Connected to Algorithm, Arduino and Android')

            # self.read_algorithm_process.start()
            self.read_STM_process.start()
            self.read_android_process.start()
            self.write_process.start()
            self.write_android_process.start()

            if self.image_process is not None:
                self.image_process.start()
                print(
                    'All processes have started : read-arduino, read-algorithm, read-android, write-android, image_processing')
            else:
                print('All processes have started : read-arduino, read-algorithm, read-android, write-android')

            print('Multiprocess communication session started')

        except Exception as error:
            raise error

        # self._allow_reconnection()

    def end(self):
        # self.algorithm.disconnect_all_algo()
        self.android.disconnect_all()
        print('Multiprocess communication has just stopped')

    def _allow_reconnection(self):
        print('RPi can reconnect now')

        while True:
            try:
                # if not self.read_algorithm_process.is_alive():
                # self._reconnect_algorithm()

                if not self.read_arduino_process.is_alive():
                    self._reconnect_arduino()

                if not self.read_android_process.is_alive():
                    self._reconnect_android()

                if not self.write_process.is_alive():
                    if self.dropped_connection.value == 0:
                        self._reconnect_arduino()
                    elif self.dropped_connection.value == 1:
                        self._reconnect_algorithm()

                if not self.write_android_process.is_alive():
                    self._reconnect_android()

                if self.image_process is not None and not self.image_process.is_alive():
                    self.image_process.terminate()

            except Exception as error:
                print("Error during reconnection: ", error)
                raise error

    # def _reconnect_algorithm(self):
    #     self.algorithm.disconnect_algo()

    #     self.read_algorithm_process.terminate()
    #     self.write_process.terminate()
    #     self.write_android_process.terminate()

    #     self.algorithm.connect_algo()

    #     self.read_algorithm_process = Process(target=self._read_algorithm)
    #     self.read_algorithm_process.start()

    #     self.write_process = Process(target=self._write_target)
    #     self.write_process.start()

    #     self.write_android_process = Process(target=self._write_android)
    #     self.write_android_process.start()

    #     print('Successfully reconnected to Algorithm'

    def _reconnect_arduino(self):
        self.arduino.disconnect_arduino()

        self.read_arduino_process.terminate()
        self.write_process.terminate()
        self.write_android_process.terminate()

        self.arduino.connect_arduino()

        self.read_arduino_process = Process(target=self._read_arduino)
        self.read_arduino_process.start()

        self.write_process = Process(target=self._write_target)
        self.write_process.start()

        self.write_android_process = Process(target=self._write_android)
        self.write_android_process.start()

        print('Successfully reconnected to Arduino')

    def _reconnect_android(self):
        self.android.disconnect_android()

        self.read_android_process.terminate()
        self.write_process.terminate()
        self.write_android_process.terminate()

        self.android.connect_android()

        self.read_android_process = Process(target=self._read_android)
        self.read_android_process.start()

        self.write_process = Process(target=self._write_target)
        self.write_process.start()

        self.write_android_process = Process(target=self._write_android)
        self.write_android_process.start()

        print('Successfully reconnected to Android')

    def _update_img_deque(self, img, message):
        self.image_deque.append([img, message])

    def _read_arduino(self):
        while True:
            try:
                messages = self.arduino.read_arduino()

                if messages is None:
                    continue
                message_list = messages.splitlines()

                for message in message_list:

                    if len(message) <= 0:
                        continue

                    print(self._format_for(ALGORITHM_HEADER,
                                           message + NEWLINE))
                    self.message_deque.append(self._format_for(
                        ALGORITHM_HEADER,
                        message + NEWLINE
                    ))

            except Exception as error:
                print('Process read_arduino failed: ' + str(error))
                break

    def _add_STM_command_to_queue(self, command):
        self.message_deque.append(self._format_for(ARDUINO_HEADER, command))

    # def _read_algorithm(self):
    #     while True:
    #         try:
    #             messages = self.algorithm.read_algo()

    #             if messages is None:
    #                 continue

    #             message_list = messages.splitlines()

    #             for message in message_list:

    #                 if len(message) <= 0:
    #                     continue

    #                 elif message[0] == AlgorithmToAndroid.MDF_STRING:
    #                     print(message + NEWLINE)
    #                     self.to_android_message_deque.append(
    #                         message + NEWLINE)

    #                 # image recognition
    #                 elif message[0] == AlgorithmToRPi.TAKE_PICTURE:

    #                     if self.image_count.value >= self.total_img_count:
    #                         self.message_deque.append(self._format_for(
    #                         ALGORITHM_HEADER, 
    #                         RPiToAlgorithm.DONE_IMG_REC + NEWLINE
    #                     ))

    #                     else:
    #                         message = message[2:-1]  # to remove 'C[' and ']'
    #                         print('tst')
    #                         image = self._take_pic()
    #                         print('Picture taken')
    #                         self.message_deque.append(self._format_for(
    #                             ALGORITHM_HEADER,
    #                             RPiToAlgorithm.DONE_TAKING_PICTURE + NEWLINE
    #                         ))
    #                         self.image_deque.append([image, message])

    #                 elif message == AlgorithmToRPi.EXPLORATION_COMPLETE:
    #                     # to let image processing server end all processing and display all images
    #                     pil_img = Image.open(STOPPING_IMAGE).convert('RGB')
    #                     cv_img = np.array(pil_img)
    #                     cv_img = cv_img[:,:,::-1].copy()
    #                     self.image_deque.append([cv_img,"-1"])

    #                 else:
    #                     #movement commands
    #                     print(self._format_for(ARDUINO_HEADER,
    #                     message))
    #                     self.message_deque.append(self._format_for(
    #                         ARDUINO_HEADER,
    #                         message
    #                     ))

    #                     decode_msg = message.decode()
    #                     if decode_msg[0] in SWAP:
    #                         decode_msg = SWAP[decode_msg[0]] + decode_msg[1:]
    #                     elif 'T' in decode_msg or 'Y' in decode_msg:
    #                         continue
    #                     print("Update Map position: ", decode_msg)
    #                     self.to_android_message_deque.append(AlgorithmToAndroid.POSITION + decode_msg.encode())

    #         except Exception as error:
    #             print('Process read_algorithm failed: ' + str(error))
    #             break

    def _read_android(self):
        while True:
            try:
                messages = self.android.read_android()

                if messages is None:
                    continue

                message_list = messages.splitlines()

                for message in message_list:
                    if len(message) <= 0:
                        continue

                    # After time limit is reached
                    elif message == AndroidToRPi.QUIT:

                        # immediately stop the movement of STM
                        self.message_deque.append(self._format_for(
                            ARDUINO_HEADER, RPiToArduino.STOP
                        ))

                        # send stop to algorithm

                        # to let image processing server end all processing and display all images
                        pil_img = Image.open(STOPPING_IMAGE).convert('RGB')
                        cv_img = np.array(pil_img)
                        cv_img = cv_img[:, :, ::-1].copy()
                        self.image_deque.append([cv_img, "-1"])

                    elif message in (AndroidToArduino.ALL_MESSAGES):
                        self.message_deque.append(self._format_for(
                            ARDUINO_HEADER, message
                        ))
                    # Android sends map data to Algorithm
                    elif AndroidToAlgorithm.SEND_ARENA in message:
                        self.message_deque.append(self._format_for(
                            ALGORITHM_HEADER, message + NEWLINE
                        ))

                        # finding out how many images that needs to be detected
                        msg_d = message.decode()
                        coord_lst = msg_d[10:].split('|')
                        self.total_img_count = len(coord_lst)
                        print("Number of images: ", self.total_img_count)

                    else:
                        if message == AndroidToAlgorithm.START_IMAGE_RECOGNITION:
                            # self.status = Status.IMAGE_RECOGNITION
                            # self.message_deque.append(self._format_for(
                            #     ARDUINO_HEADER,
                            #     RPiToArduino.START_IMAGE_RECOGNITION + NEWLINE
                            # ))
                            self.task_2.run()
                        elif message in AndroidToAlgorithm.START_FASTEST_PATH:
                            # self.status = Status.FASTEST_PATH
                            # time.sleep(0.5)
                            # self.message_deque.append(self._format_for(
                            #     ARDUINO_HEADER, 
                            #     RPiToArduino.START_FASTEST_PATH + NEWLINE
                            # ))
                            self.task_2.run()



            except Exception as error:
                print('Process read_android failed: ' + str(error))
                break

    def _write_target(self):
        while True:
            target = None
            try:
                if len(self.message_deque) > 0:
                    message = self.message_deque.popleft()
                    target, payload = message['target'], message['payload']
                    print("payload: ", payload)

                    if target == ARDUINO_HEADER:
                        self.arduino.write_arduino(payload)

                    elif target == ALGORITHM_HEADER:
                        self.algorithm.write_algo(payload)

                    else:
                        print("Invalid header", target)

                time.sleep(0.05)

            except Exception as error:
                print('Process write_target failed: ' + str(error))

                if target == ARDUINO_HEADER:
                    self.dropped_connection.value = 0

                elif target == ALGORITHM_HEADER:
                    self.dropped_connection.value = 1

                self.message_deque.appendleft(message)

                break

    def _read_STM(self):
        while True:
            try:
                messages = self.arduino.read_arduino()

                if messages is None:
                    continue

                message_list = messages.splitlines()

                for message in message_list:

                    message = message.decode()
                    if len(message) <= 0:
                        continue

                    if constants.ROBOT_COMMAND_COMPLETED in message:
                        self.task_2.set_command_complete_status(True)

                    if constants.ROBOT_READY_TO_PARK in message:
                        self.task_2.set_is_robot_ready_to_park(True)

            except Exception as error:
                print('Process read_STM failed: ' + str(error))
                break

    def _write_android(self):
        while True:
            try:
                if len(self.to_android_message_deque) > 0:
                    message = self.to_android_message_deque.popleft()

                    self.android.write_android(message)

            except Exception as error:
                print('Process write_android failed: ' + str(error))
                self.to_android_message_deque.appendleft(message)
                self._reconnect_android()
                break

    def _take_pic(self):
        global img_count
        try:
            print('taking photo')
            # initialise pi camera
            camera = PiCamera(resolution=(IMAGE_WIDTH, IMAGE_HEIGHT))
            camera.awb_mode = 'horizon'

            # allow the camera to warmup
            time.sleep(3)
            rawCapture = PiRGBArray(camera)

            # grab an image from the camera
            camera.capture(rawCapture, format='bgr')
            image = rawCapture.array
            pil_img = np.asarray(image)
            pil_img = Image.fromarray(pil_img[:, :, ::-1])
            pil_img.save('./frame_{}.jpg'.format(img_count))
            img_count += 1

            camera.close()

        except Exception as error:
            print('Picture taking process failed: ' + str(error))

        return image

    def _process_pic(self):
        print("----------------------- Image Processing Starts -------------------")
        print(self.image_processing_server_url)
        image_sender = imagezmq.ImageSender(
            connect_to=self.image_processing_server_url)
        print(type(self.image_deque))
        image_id_list = []
        while True:
            try:
                if not self.image_deque.empty():

                    message_for_image = self.image_deque.popleft()
                    target_id = message_for_image[1]
                    print("-----------Preparing to send image---------")

                    if target_id == 'C':
                        reply = image_sender.send_image(
                            'image message from RPi' + target_id,
                            message_for_image[0]
                        )

                    else:
                        reply = image_sender.send_image(
                            'image message from RPi',
                            message_for_image[0]
                        )
                    reply = reply.decode('utf-8')
                    print("Reply:" + reply)

                    if reply == 'End':
                        break  # stop sending images

                    else:
                        detection = reply.split('$')[0]  # from server
                        offset = reply.split('$')[1]

                        if detection == '-1':
                            continue  # if there isn't any  symbol detected, mapping of symbol id will  be skipped

                        elif detection == '41':
                            # self.message_deque.append(self._format_for(
                            #    ALGORITHM_HEADER,
                            #    RPiToAlgorithm.BULLSEYE + NEWLINE
                            # ))
                            continue  # if the image is a bullseye, robot needs to navigate around the obstacle

                        elif target_id == '-1':
                            continue  # if there isn't any obstacle detected, mapping of id symbol will skipped

                        elif target_id == 'C':  # error correction
                            dist = distance()  # ultrasonic dist
                            message = 'O|' + offset + '$' + str(dist)
                            # self.message_deque.append(self._format_for(
                            #    ALGORITHM_HEADER,
                            #    message.encode() + NEWLINE
                            # ))


                        else:
                            self.cur_img_result = detection
                            self.task_2.set_img_result(detection)

                            """
                            id_string_to_android = 'IM|'+ target_id.decode() + ',' + detection
                            print(id_string_to_android)
                            
                            if detection not in image_id_list:
                                self.image_count.value += 1
                                image_id_list.append(detection)
                            
                            self.to_android_message_deque.append(
                                id_string_to_android.encode() + NEWLINE
                            )
                            time.wait(0.1)
                            """

            except Exception as error:
                print('Image processing process has failed: ' + str(error))
                time.wait(2)

    def _format_for(self, target, payload):
        return {
            'target': target,
            'payload': payload,
        }

    def _run_task2(self):
        self.task_2.run()

        # image recognition termination
        pil_img = Image.open(STOPPING_IMAGE).convert('RGB')
        cv_img = np.array(pil_img)
        cv_img = cv_img[:, :, ::-1].copy()
        self.image_deque.append([cv_img, "-1"])
