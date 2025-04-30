Hello, this is just quick documentation regarding the software/Arduino code <br>
**PLEASE NOTE THAT THIS PAYLOAD IS STILL IN SUPER EARLY STAGES. THIS CODE BASICALLY ASSUMES THE PAYLOAD WILL BE A TESTBENCH/IN A THROUGHHOLE BOX**
# Setup
To get this working on your local machine
- [STM32 Cube Programmer] (https://www.st.com/en/development-tools/stm32cubeprog.html)
  - In Windows, make sure Cube Programmer is in your environment/path variables
- All of the libraries in the [libraries folder](https://github.com/UMDBPP/GHOUL/tree/CloudCruiser_Fall2024/Code/libraries) of this branch
- [Follow this tutorial](https://community.st.com/t5/stm32-mcus/how-to-program-and-debug-the-stm32-using-the-arduino-ide/ta-p/608514) 

From there, before you upload the code to the uCon, short the red and white wires (Power and Bootloader pin) and press the button to reset the uCon to bootloader (it should disconnect from your PC)
# The code
Will give a tl;dr of the code, I hope I do a decent job explaining. General notes
- We have definitions ot ms(default) to minutes, feet to mm, altitude thresholds for cutdown, logging to SD card, float duration, flight duration, etc.
## Setup
- 4 GPIO pins for motor
- Initialize I2C, SD card, txt file for writing, GPS, Barometer
- Infinite loop where it looks for satellies in view, code will not progress until GPS has LOS with at least 1 satellite
## Loop
- Get altitude for PID
- Get Pressure and temp
- Check for cutdown conditions
- Run PID
- If above altitude threshold, log to SD card
  - The flush and buffer stuff is just to ensure we keep consistently writing to the end of the file on the SD card
- After 2 hours, we basically "turn off" the payload, close the text file
## triggerCutdown
- Set the cutdown pin high
- Mark the cutdown flag high for SD card writing
## adjustValve
- Takes only PID output as arguement
  -Based on output value, will either not vent, vent for 5 seconds, or vent for 10 seconds
## logtoSD
Appends all recorded values to buffer, once buffer reaches limit (256 char), we will write to the SD card using flushlog
## flushLog
Seeks to the end of the file, writes all the contents in the buffer, closes the file, clears the buffer!
## alt_check
This is cutdown condition function. Basically how it works is theres a counter that goes up when altitude goes to high. When the count exceeds a threshold, we return cut, if it goes back down to acceptable altitudes, count goes back to 0.
## moveForward/moveBackward/stop Motor
GPIO writes to move the motors. If the motor doesn't move, comment out lines 249,251,255,257 (all the write lows in the move functions)
