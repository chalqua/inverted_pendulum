from time import sleep
from time import monotonic_ns
import board
import busio
import adafruit_mpu6050
import math
import numpy as np

##Setup accelerometer
i2c = busio.I2C(board.SCL, board.SDA)
mpu = adafruit_mpu6050.MPU6050(i2c)

# Calibration Value
RateCalibrationPitch = 0
RateCalibrationNumber = 0

# Calibration Value
AngleCalibration = 0
RateCalibrationPitch = 0

#Inital state of the system
RatePitch = 0
AnglePitch = 0
Comp_angle = 0

def Angle_Calc():
   global RatePitch, AnglePitch

   # Read necessary Accelerometer raw values
   acc_x, acc_y, acc_z = mpu.acceleration # In m/s^2

   # Read necessary Gyroscope raw value
   Gyro_x, Gyro_y, Gyro_z = mpu.gyro #radian.s^-1
   RatePitch = -(Gyro_y - RateCalibrationPitch)

   # Angle Calculated with Accelerometer
   AnglePitch = math.atan2(-acc_x, -acc_z)

def Gyro_Calibrations():
   global RateCalibrationPitch, RateCalibrationNumber
   # Calibrate the gyroscope
   for RateCalibrationNumber in range(2000):
       RatePitch = mpu.gyro[1]
       RateCalibrationPitch += RatePitch
       sleep(0.001)

   # Average the calibration values
   RateCalibrationPitch /= 2000

#Gyro weight
Gyro_weight = 0.95
def Comp_Angle_calc(Angle, Gyro, IT_time, Acc_angle):
   return Gyro_weight*(Angle+Gyro*IT_time) + (1-Gyro_weight)*Acc_angle

def Angle_Calib2():
   global Comp_angle, AngleCalibration
   for _ in range(2000):
       IT_START = monotonic_ns()*1e-9

       sleep(0.001)
       #Iteration time
       IT_time = monotonic_ns()*1e-9 - IT_START

       # Read accelerometer raw values
       acc_x, acc_y, acc_z = mpu.acceleration

       # Read gyroscope values
       gyro_y = -(mpu.gyro[1] - RateCalibrationPitch)

       # Angle from accelerometer
       Angle_Pitch = np.atan2(-acc_x, -acc_z)

       # Calculate angle with complementary filter
       Comp_angle = Comp_Angle_calc(Comp_angle, gyro_y, IT_time, Angle_Pitch)

       AngleCalibration+= Comp_angle
   AngleCalibration /= 2000

Angle_Calc()
Comp_Angle = AnglePitch

Gyro_Calibrations()
print("Gyro Calibration:", RateCalibrationPitch)

Angle_Calib2()
print('Angle calibration: ', AngleCalibration)
