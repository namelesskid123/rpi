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
ARDUINO_HEADER = 'ARD'.encode()
ALGORITHM_HEADER = 'ALG'.encode()

class Status:
    IDLE = 'idle'.encode()
    IMAGE_RECOGNITION = 'IR'.encode()
    FASTEST_PATH = 'fastest path'.encode()

class AlgorithmToArduino:
    MOVE_FORWARD = 'F1|'.encode()
    STOP = 'F2|'.encode()
    REVERSE = 'F3|'.encode()
    RIGHTWIDE =  'R1|'.encode()
    RIGHTNARROW = 'R2|'.encode()
    LEFTWIDE = 'L1|'.encode()
    LEFTNARROW = 'L2|'.encode()
    CENTER = 'C1|'.encode()
    
class AndroidToArduino:
    FORWARD = 'F1'.encode()
    STOP = 'F2'.encode()
    REVERSE = 'F3'.encode()
    RIGHT_WIDE = 'R1'.encode()
    RIGHT_NARROW = 'R2'.encode()
    LEFT_WIDE = 'L1'.encode()
    LEFT_NARROW = 'L2'.encode()
    CENTER = 'C1'.encode()

    ALL_MESSAGES = [
        FORWARD,
        STOP,
        REVERSE,
        RIGHT_WIDE,
        RIGHT_NARROW,
        LEFT_WIDE,
        LEFT_NARROW,
        CENTER
    ]


class AndroidToAlgorithm:
    START_IMAGE_RECOGNITION = 'IR|'.encode()
    START_FASTEST_PATH = 'FP|'.encode()
    SEND_ARENA = 'SendArena|'.encode()
    REQUEST_MDF = 'MDF|'.encode()

class AndroidToRPi:
    CALIBRATE_SENSOR = 'SC|'.encode()

class AlgorithmToArduino:
    FORWARD = 'F1|'.encode()
    STOP = 'F2|'.encode()
    REVERSE = 'F3|'.encode()
    RIGHT_WIDE = 'R1|'.encode()
    RIGHT_NARROW = 'R2|'.encode()
    LEFT_WIDE = 'L1|'.encode()
    LEFT_NARROW = 'L2|'.encode()
    CENTER = 'C1|'.encode()


class AlgorithmToAndroid:
    MOVE_FORWARD = 'FS'.encode()[0]
    TURN_LEFT = 'L'.encode()[0]
    TURN_RIGHT = 'W'.encode()[0]
    CALIBRATING_CORNER = 'C'.encode()[0]
    SENSE_ALL = 'S'.encode()[0]
    ALIGN_RIGHT = 'AR'.encode()[0]
    ALIGN_FRONT = 'AF'.encode()[0]

    MDF_STRING = 'M'.encode()[0]


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
    
    IMAGE = 'IM|'
    POSITION = 'P|'


class RPiToArduino:
    CALIBRATE_SENSOR = 'SC|'.encode()
    START_IMAGE_RECOGNITION = 'E|'.encode()
    START_FASTEST_PATH = 'F|'.encode()


class RPiToAlgorithm:
    DONE_TAKING_PICTURE = 'D'.encode()
    DONE_IMG_REC = 'I'.encode()

class Rubbish:
    Rubbish = '\x00'.encode()
