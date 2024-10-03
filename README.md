# Time of Flight Spatial Scanner

This project was my final submission for COMPENG 2DX3. The system is designed around a **MSP432E401Y microcontroller** and is capable of scanning and creating a 3D map of its surroundings. It operates by rotating a **time of flight (ToF) sensor** with a **stepper motor**, collecting distance measurements every 22.5 degrees. The data from the time of flight sensor is sent to the microcontroller using **I2C** serial communication, and from there sent to a PC using **UART**. The data is then processed and mapped using **Open3D** in Python. 

This project showcases the integration of **hardware design**, **serial communication**, and **3D data visualization** to create a functional time of flight spatial scanner. 

A more detailed description of the project can be found in [Spatial Scanner Datasheet](Spatial_Scanner_Datasheet.pdf)
