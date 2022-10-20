"""algo -> rpi: long string of direction
rpi -> stm: broken down direction to stm
rpi -> algo: once direction msg ends, send msg to algo 
"""

'''
Communication protocols.
They are pre-defined so it allows all subsystems to know the common ways of communication
'''

MESSAGE_SEPARATOR = '|'.encode()
NEWLINE = '\n'.encode()

ANDROID_HEADER = 'AND'.encode()
STM_HEADER = 'ARD'.encode()
ALGORITHM_HEADER = 'ALG'.encode()

class Status:
    IDLE = 'idle'.encode()
    IMAGE_RECOGNITION = 'IR'.encode()
    FASTEST_PATH = 'fastest path'.encode()

class AlgorithmToSTM:
    FORWARD = 'F010'.encode()
    REVERSE = 'Z010'.encode()
    RIGHT = 'R090'.encode()
    BACK_RIGHT = 'E097'.encode()
    LEFT = 'L090'.encode()
    BACK_LEFT = 'K097'.encode()

class AndroidToSTM:
    FORWARD = 'F010'.encode()
    REVERSE = 'Z010'.encode()
    RIGHT = 'R090'.encode()
    BACK_RIGHT = 'E097'.encode()
    LEFT = 'L090'.encode()
    BACK_LEFT = 'K097'.encode()

    ALL_MESSAGES = [
        FORWARD,
        REVERSE,
        RIGHT,
        BACK_RIGHT,
        LEFT,
        BACK_LEFT
    ]


class AndroidToAlgorithm:
    START_IMAGE_RECOGNITION = 'IR|'.encode()
    START_FASTEST_PATH = 'FP|'.encode()
    SEND_ARENA = 'SendArena|'.encode()
    REQUEST_MDF = 'MDF|'.encode()

class AndroidToRPi:
    CALIBRATE_SENSOR = 'SC|'.encode()
    QUIT = 'QUIT'.encode()

class AlgorithmToAndroid:
    MOVE_FORWARD = 'FS'.encode()[0]
    TURN_LEFT = 'L'.encode()[0]
    TURN_RIGHT = 'W'.encode()[0]
    CALIBRATING_CORNER = 'C'.encode()[0]
    SENSE_ALL = 'S'.encode()[0]
    ALIGN_RIGHT = 'AR'.encode()[0]
    ALIGN_FRONT = 'AF'.encode()[0]

    MDF_STRING = 'M'.encode()[0]
    IMAGE = 'IM|'.encode()
    POSITION = 'P|'.encode()


class AlgorithmToRPi:
    TAKE_PICTURE = 'C'.encode()[0]
    EXPLORATION_COMPLETE = 'N'.encode()


class RPiToAndroid:
    STATUS_EXPLORATION = '{"status":"exploring"}'.encode()
    STATUS_SHORTEST_PATH = '{"status":"shortest path"}'.encode()
    STATUS_TURN_LEFT = '{"status":"turning left"}'.encode()
    STATUS_TURN_RIGHT = '{"status":"turning right"}'.encode()
    STATUS_IDLE = '{"status":"idle"}'.encode()
    STATUS_TAKING_PICTURE = '{"status":"taking picture"}'.encode()
    STATUS_CALIBRATING_CORNER = '{"status":"calibrating corner"}'.encode()
    STATUS_SENSE_ALL = '{"status":"sense all"}'.encode()
    STATUS_MOVING_FORWARD = '{"status":"moving forward"}'.encode()
    STATUS_ALIGN_RIGHT = '{"status":"align right"}'.encode()
    STATUS_ALIGN_FRONT = '{"status":"align front"}'.encode()

    MOVE_UP = '{"move":[{"direction":"forward"}]}'.encode()
    TURN_LEFT = '{"move":[{"direction":"left"}]}'.encode()
    TURN_RIGHT = '{"move":[{"direction":"right"}]}'.encode()


class RPiToSTM:
    CALIBRATE_SENSOR = 'SC|'.encode()
    START_IMAGE_RECOGNITION = 'E|'.encode()
    START_FASTEST_PATH = 'F|'.encode()
    STOP = 'STOP'.encode()


class RPiToAlgorithm:
    BULLSEYE = 'B'.encode()
    DONE_TAKING_PICTURE = 'D'.encode()
    DONE_IMG_REC = 'I'.encode()

SWAP = {
'F' : 'Z',
'Z' : 'F',
'E' : 'R',
'R' : 'E',
'K' : 'L',
'L' : 'K'
}
    
