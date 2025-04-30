# Main PCB
Here's my attempt of a markdown file for documentation. Here goes!
![image](https://github.com/user-attachments/assets/9d11ba25-4d68-48c7-bfec-d754faeea8fe)
![image](https://github.com/user-attachments/assets/7cf9e19c-3246-4611-9658-f38103a168e5)

## Adafruit STMFeatherF405 Express uCon
This is a custom symbol/footprint that I made, so hopefully it's here in this repo. If not, just know that it's a 28 pin symbol and you can refer to [This link](https://www.adafruit.com/product/4382) for pinouts and more about it
![image](https://github.com/user-attachments/assets/7ea131e5-d7c4-4e2e-a331-0a0cd485edd1)
## Motor Driver
I forgot the name of the specific motor driver on the PCB, but it's a dual H-Bridge that can control up to 2 stepper motors. **As of right now, this payload uses a servo motor, and is powered through GPIO pins on the uCon**
## GNSS Module MAX M10 GPS
This was pulled from GHOUL. Just know that this is on the I2C bus. Refer to [This Link](https://www.u-blox.com/en/product/max-m10-series) for more info
![image](https://github.com/user-attachments/assets/d039b5b4-1764-4408-81de-69029dfd0ffc)
## Barometer
This is also in the I2C bus. Refer to [This Link](https://www.digikey.com/en/products/detail/te-connectivity-measurement-specialties/MS560702BA03-50/4700921) for more info/anything regarding the schematic
![image](https://github.com/user-attachments/assets/445e580c-5636-487e-8d67-0e9bd47be9a7)
## Everything else
Everything else was basically pulled from GHOUL, which includes the external battery connector, the voltage switcher/regulator component (which just converts external 9V to 5V/3.3V logic for uCon), an XBee radio, temperature probes, and some MOSFET circuits (for the heatpads and Nichrome strips). <br>
The last thing to note is that there's a 3 pin connector for the SPI bus and a connector for the 2nd PCB
# Bert Board
Named after Robert Heng, thanks Robert :) <br>
Baby Bert Board has a barometer (basically same schematic as main PCB), and a 3 pin connector to go back to main PCB. As of 4/29/25, this connection has yet to be soldered.
![image](https://github.com/user-attachments/assets/a0d0c706-5957-40ae-8d9e-46e11ecbcce3)
![image](https://github.com/user-attachments/assets/df610f99-bf50-452f-83c5-f997a764e17c)
