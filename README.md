# FOC-Driver
A modified version of the SimpleFOC based project. The official design was a bit large for my project below, so I redesigned the FOC drive and made it small enough (42mm x 42mm) to fit a small BLDC motor as well as 42 step motor.


## FOC-Driver v1.0
The 1.0 version is just a testing version. Progress has been slow due to a number of constraints, including funding and logistics. Because it is a testing version, I add a small OLED screen to display the current state. 

<img src="https://github.com/Xiangyu-Fu/FOC-Driver/assets/54738414/f85f9efc-5d84-4a6f-bd46-b32a8494b932" width="450" >

<img src="https://github.com/Xiangyu-Fu/FOC-Driver/assets/54738414/6ad3d854-6827-4592-9b19-2b2ce9215eda" width="450" >

But the design of the circuit has some bugs that the inductor has burned. 

## FOC-Driver v1.1

New PCB design (3D version):

<img src="https://github.com/Xiangyu-Fu/FOC-Driver/assets/54738414/9f86dd3f-7c60-4381-9617-4017c6ae9b48" width="450" >

Now add a new convert circuit to insulate the different power supply source.

![image](https://github.com/Xiangyu-Fu/FOC-Driver/assets/54738414/51ef5153-8091-495a-86af-5ccee8b08160)

> But it still have some bugs.
