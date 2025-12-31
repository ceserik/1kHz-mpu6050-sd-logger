# 1khz logger for mpu6050 accelerometer + gyro, using arduino mini pro
I wrote this to log motion of hobby rocket launch. The code periodically reads FIFO buffer on MPU6050 and logs it to a SD card. Maximum sample rate of accelerometer is 1kHz, so I chose this as logging frequency. The data is stored as binary on sd card and you need to use converter.py to convert it to csv.

## Parts list
-5V/16MHz Arduino mini pro 
-5V compatible SD adapter 
-MPU6050 
-300mAh lipo (lasts for around 6 hours, which is overkill) 
-5V stepup 
-3d printed holder


## limitations
You have to use max 2GB sd card formatted as fat16. Larger sd cards will result in FIFO overflow around every 20 seconds, for about 60 milliseconds. This  should be fixable by better utolisation of SDfat library.
