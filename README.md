# stm32-appLauncher
Application launcher using input from an accel. and a stm32 board micro.

Reading the data from the accelerometer, the microcontroller sends data to the computer which opens a determined app according to the orientation of said accelerometer.

ADXL345 sends data via I2C to the STM32, which processes the data and can display the graphs of the X, Y and Z axis acceleration values over time on the built-in display. 
The microcontroller also sends the values via UART to the computer. A program written in Python reads the data and opens the corresponding application:
- If tilted left: Open Zoom
- If tilted back: Open SofaScore
- If tilted right: Open Chrome
