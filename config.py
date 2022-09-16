LOCALE = 'UTF-8'

#Android BT connection settings

RFCOMM_CHANNEL = 8
RPI_MAC_ADDR = "B8:27:EB:14:A1:9C"
UUID = "00001101-0000-1000-8000-00805F9B34FA"
ANDROID_SOCKET_BUFFER_SIZE = 512

# Algorithm Wifi connection settings
# raspberryHotPotato: 192.168.3.1
WIFI_IP = "192.168.14.14"
WIFI_PORT = 3054
ALGORITHM_SOCKET_BUFFER_SIZE = 512

# Arduino USB connection settings
# SERIAL_PORT = '/dev/ttyACM0'
# Symbolic link to always point to the correct port that arduino is connected to
SERIAL_PORT = "/dev/ttyUSB0" #"/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0002-if00-port0" 
BAUD_RATE = 115200

# Image Recognition Settings
STOPPING_IMAGE = 'stop_image_processing.jpg'

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 360
IMAGE_FORMAT = 'bgr'

BASE_IP = 'tcp://192.168.3.'
PORT = ':5555'

IMAGE_PROCESSING_SERVER_URLS = {
    'cheyanne': BASE_IP + '1' + PORT,
    'elbert': BASE_IP + '00' + PORT,  # don't have elbert's ip address yet
    'jason': BASE_IP + '52' + PORT,
    'joshua': BASE_IP + '93' + PORT,
    'mingyang': BASE_IP + '74' + PORT,
    'reuben': BASE_IP + '00' + PORT,  # don't have reuben's ip address yet
    'winston': BASE_IP + '55' + PORT,
    'yingting': BASE_IP + '90' + PORT,
}

'''
=======
#from seniors
BASE_IP = 'tcp://192.168.3.'
PORT = ':5555'

IMAGE_PROCESSING_SERVER_URLS = 'tcp://192.168.3.13:5555'
>>>>>>> f2afaf36a2b19dd5407c0283ddd18d146ad6b9d5
'''

