# rpi
rpi
Raspberry Pi Codebase for CZ3004 MDP

Ensure that OpenCV 3.3.0 is installed

## Set Up on Raspberry Pi
   Working on Raspbian Jessie (2016-02-26), with Python version 3.4.2. Ensure that OpenCV 3.3.0 is installed as well.

   Ensure that `picamera` version is at 1.1.0 (`sudo pip3 install "picamera[array]" == 1.1.0`)
   
   Ensure that `at-spi2-core` is installed (`sudo apt-get install at-spi2-core`)

## To run
   
- Before running main program, open another terminal and SSH into rpi.

- then type 'bluetoothctl' and 'discoverable on'

- Main Program: `sudo python3 -m main`

   Begins a multithread session that will establish communications with N7 Tablet, Arduino and PC. Also starts a video stream that will attempt to detect symbols in front of it. Program will conclude the ID of the detected symbol depending on an arbitrary threshold value.
   Still a work in progress - multithread communication is commented out for now to test the detection alone.

## Connecting a new bluetooth device
`sudo hciconfig hci0 piscan`

`hcitool scan`
