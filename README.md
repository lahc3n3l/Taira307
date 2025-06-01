# ESP 32 Flight Controller
___

![image](esp32-devkitC-v4-pinout.png)


### PIN DEFINITION:
* Servos:
    * left servo: 13 
    * right servo: 12 
    * pitch servo: 14

* FlySky Reciever:
    * Roll: 2 
    * Pitch: 4
    * Flaps: 5

* GPS:
    * sda: 16
    * scl: 17

* MPU5060: 
    * sda: 36
    * scl: 35

# The general architecture used to perform autonomous missions is the following

![gncArchitecture](image.png)


# run and upload a specif target to the board:
pio run -e name_of_env -t upload

example : 
pio run -e goouuu -t upload
