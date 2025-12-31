# 1khz logger for mpu6050 accelerometer + gyro, using arduino mini pro
I wrote this to log motion of hobby rocket launch. The code periodically reads FIFO buffer on MPU6050 and logs it to a SD card. Maximum sample rate of accelerometer is 1kHz, so I chose this as logging frequency. The data is stored as binary on sd card and you need to use converter.py to convert it to csv.

## Parts list

- 5V/16MHz Arduino mini pro 
- 5V compatible SD adapter 
- MPU6050 
- 300mAh lipo (lasts for around 6 hours, which is overkill) 
- lipo protection/charging module
- 5V stepup 
- 3d printed holder(Freecad + .stp files in repo)

## How to use
1. Wire everyting up
2. Insert sd card
3. Power on
4. Wait for the calibration of accelerometer and gyros, the MPU6060 should be perfecly still and level with ground (Z axis of MPU6050 pointing directly up) 
5. Led on arduino should start blining fsat and that indicates logging
6. The file size pre allocated on sd card is 9.5MB which is enough for around 10 minutes of logging, after that more size is automatically allocated but it may cause overflow, i havent checked.
7. Power of
8. Remove sd card and put it into pc
9. use converter.py to convert binary to csv
10.???

## limitations
You have to use max 2GB sd card formatted as fat16. Larger sd cards will result in FIFO overflow around every 20 seconds, for about 60 milliseconds. This  should be fixable by better utilisation of SDfat library.
