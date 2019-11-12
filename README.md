# arduino-bicycle-security-system
My project for "Step into the future 2017" made in 2016.

Basic description of it's work process:
1. User locks the system using IC 2262/2272 remote control.
2. System goes to the "blocked" mode - gyro GY-BMI160 starts tracking the current angle. If the angle changes more than on 45 degrees on any axis, then move to the step 3.
3. GPS module GTPA010 starts tracking the location and if changes goes to the step 4.
4. GSM/GPRS module SIM 800L with SIM-card inside starts sending messages to the owner with current location of device.
5. User can unblock the system on any stage and get it into sleeping mode using the remote control.

System allows to track the stolen bicycle (or any other thing it was attached to, if it provides the opportunity for GPS module to connect to the sattelites). All the components could be replaced with their analogues, in some cases it would require changing the code (another libraries, constants etc.). Total cost was ~2000 rubles in 2016. Body was 3D printed.

Scheme of the device:
![alt-текст](https://github.com/uberpup/arduino-bicycle-security-system/blob/master/img1.png "Prototype Scheme")

Body:
![alt-текст](https://github.com/uberpup/arduino-bicycle-security-system/blob/master/img2.png "Body in Autodesk")

The look:
![alt-текст](https://github.com/uberpup/arduino-bicycle-security-system/blob/master/img3.png "Look inside")
