# rpi
rpi
Raspberry Pi Codebase for CZ3004 MDP

Ensure that OpenCV 3.3.0 is installed

## Set Up on Raspberry Pi
   Working on Raspbian Jessie (2016-02-26), with Python version 3.4.2. Ensure that OpenCV 3.3.0 is installed as well.

   Ensure that `picamera` version is at 1.1.0 (`sudo pip3 install "picamera[array]" == 1.1.0`)
   
   Ensure that `at-spi2-core` is installed (`sudo apt-get install at-spi2-core`)

## To run
   
- Main Program: `sudo python3 -m main`

   Begins a multithread session that will establish communications with N7 Tablet, Arduino and PC. Also starts a video stream that will attempt to detect symbols in front of it. Program will conclude the ID of the detected symbol depending on an arbitrary threshold value.
   Still a work in progress - multithread communication is commented out for now to test the detection alone.

## Connecting a new bluetooth device
`sudo hciconfig hci0 piscan`

`hcitool scan`


## NOTE TO ANDROID/ALGO/STM TEAM:

- the scripts in this repo is outdated, i forgot to safe a copy of the latest version of script 

- the script is in the RPI

- commands.txt is a text file with most of the message formats. Tell me if more is needed 
