#!/usr/bin/env python

from time import sleep
from time import time
import math
import numpy as np
import RPi.GPIO as GPIO
from AngularVelocityController import *
import board
import busio
import adafruit_mpu6050
import matplotlib.pyplot as plt
import logging

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO, format='%(asctime)s %(message)s')


##Define stepper motors
#define GPIO pins
GPIO_pins = (16, 20, 21) # Microstep Resolution MS0-MS2 (-1,-1,-1) if not used
DIRECTION = 27 # Direction (DIR) GPIO Pin
STEP = 17 # Step GPIO Pin
EN_PIN = 23 # enable pin (LOW to enable)
Step_Modes_Min_Delay = np.array([1300, 700, 500, 300, 160, 80]) #In micro seconds

# Remove warnings
GPIO.setwarnings(False)

#Set how pin are called
GPIO.setmode(GPIO.BCM)

#Set motor for Angular Velocity Controller
Motor = Angular_Velocity_Controller(Step_Modes_Min_Delay, DIRECTION, STEP, GPIO_pins)
GPIO.setup(EN_PIN,GPIO.OUT) # set enable pin as output

##Setup accelerometer
i2c = busio.I2C(board.SCL, board.SDA)
mpu = adafruit_mpu6050.MPU6050(i2c)

# Iterations counter
Iteration = 0

#Constants
L = 33.019e-3 #metre
m = 0.583 #kg
M = 2*0.011 # kg
g = 9,81 #m.s-2
Lxx = 829.876e-6 #kg.m^2
R_wheel = 35e-3 #m

# Targeted iteration time in seconds
IT_TIME_TARGET = 0.05
IT_time = IT_TIME_TARGET

# PID controller constants for tuning
K_P = -57# It_time=0.1 ~ -14 Théorique -135.76
K_I = 10# It_time=0.1 ~ -2.3  Théorique -681.377
K_D = 0# Théorique -5.896

# Initial values for PID
error = 0
error_prev = error
integral = 0
derivative = 0
x_pp = 0

# Set targeted angle in radians
TARGET = 0

# Abandon :(
safe = [-30*np.pi/180+TARGET, 30*np.pi/180+TARGET]

# Calibration Value
Angle_Calibration = 0.021538374780850512
RateCalibrationPitch = 0.017875592801939334

#Inital state of the system
RatePitch = 0
AnglePitch = 0
KalmanAnglePitch = 0
KalmanUncertaintyAnglePitch = 4
Mod_AnglePitch = 0
Mod_KalmanAnglePitch = 0
Mod_KalmanUncertaintyAnglePitch = 4
KalmanOutput = [0,0]

# Prepare graphs
Datasave = True # True to save data
fig = plt.figure()
t = [0]
Gyro_angles = [0]
Acc_angles = []
Mod_Acc_angles = []
Kalman_angles = []
Mod_Kalman_angles = []
Acc_raw = [[],[]]
Mod_Acc_raw = [[],[]]
Gyro_raw = []
X_Acc_l = [0]
IT_TIMES_l = [0]

# Kalman filter function
def kalman_1d(KalmanState, KalmanUncertainty, KalmanInput, KalmanMeasurement, IT_TIME):
    # Predict step
    KalmanState = KalmanState + IT_TIME * KalmanInput
    KalmanUncertainty = KalmanUncertainty + IT_TIME * IT_TIME * 4 * 4
    
    # Update step
    KalmanGain = KalmanUncertainty / (KalmanUncertainty + 3 * 3)
    KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState)
    KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty
    
    # Store the output
    KalmanOutput[0] = KalmanState
    KalmanOutput[1] = KalmanUncertainty

def Angle_Calc(x_pp):
    global RatePitch, AnglePitch, Acc_raw, Gyro_raw ,Mod_Acc_raw, Mod_KalmanAnglePitch, Mod_AnglePitch

    # Read necessary Accelerometer raw values
    acc_x, acc_y, acc_z = mpu.acceleration # In m/s^2

    # Remove x_pp from the accelerations that the accelerometer computes and inversing the reference (le repère)
    Mod_acc_x = (acc_x - np.cos(Mod_KalmanAnglePitch)*x_pp)
    Mod_acc_z = (acc_z - np.sin(Mod_KalmanAnglePitch)*x_pp)

    # Read necessary Gyroscope raw value
    Gyro_x, Gyro_y, Gyro_z = mpu.gyro #radian.s^-1
    RatePitch = -(Gyro_y - RateCalibrationPitch)

    # Angle Calculated with Accelerometer
    Mod_AnglePitch = math.atan2(-Mod_acc_x,-Mod_acc_z) - Angle_Calibration
    AnglePitch = math.atan2(-acc_x, -acc_z) - Angle_Calibration

    # To draw beautiful graphs
    Acc_raw[0].append([-acc_x])
    Acc_raw[1].append([-acc_z])
    Mod_Acc_raw[0].append([-Mod_acc_x])
    Mod_Acc_raw[1].append([-Mod_acc_z])
    Gyro_raw.append(RatePitch)

# Intiate angle to prevent sudden acceleration
Angle_Calc(0)
KalmanAnglePitch = AnglePitch
Mod_KalmanAnglePitch = Mod_AnglePitch
Acc_angles.append(AnglePitch)
Mod_Acc_angles.append(Mod_AnglePitch)
Kalman_angles.append(KalmanAnglePitch)
Mod_Kalman_angles.append(Mod_KalmanAnglePitch)

# Set while loop start time (for graph)
START_TIME = monotonic_ns()*1e-9

# Pull enable to low to enable motor
GPIO.output(EN_PIN,GPIO.LOW)

try: # Main program loop
    while Iteration < 2000:
        logger.debug("Starting iteration %s" % Iteration)
        IT_START = monotonic_ns()*1e-9

        # Command
        logger.debug("Activating motors to move at %s m.s^-2" % x_pp)
        if abs(x_pp) < 0.01:
            logger.warning("Go sleep for %s" % IT_TIME_TARGET)
            sleep(IT_TIME_TARGET)
        else:
            ang_acc = x_pp/R_wheel
            ang_velocity = IT_TIME_TARGET*ang_acc
            Step_Mode, Step_Delay, N_Step = Motor.Vel_Time_Motor_Go(ang_velocity, IT_TIME_TARGET)

        #Iteration Time
        IT_time = monotonic_ns()*1e-9-IT_START
        logger.debug("Iteration time %s" % IT_time)

        # Uptade angle calculated with accelerometer and angular velocity
        Angle_Calc(x_pp)
        logger.debug("Calculating system angle in radians: Angle_Pitch = %s, RatePitch = %s", AnglePitch, RatePitch)

        # Angle Calculated with Kalman Filter
        kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch, IT_time) # Update KalmanOutput
        KalmanAnglePitch, KalmanUncertaintyAnglePitch = KalmanOutput

        # Moded Angle Calculated with Kalman Filter
        kalman_1d(Mod_KalmanAnglePitch, Mod_KalmanUncertaintyAnglePitch, RatePitch, Mod_AnglePitch, IT_time) # Update KalmanOutput
        Mod_KalmanAnglePitch, Mod_KalmanUncertaintyAnglePitch = KalmanOutput

        # Calculate the PID terms
        error = TARGET - KalmanAnglePitch #radians
        integral = integral + (error*IT_time)
        derivative = (error - error_prev)/IT_time
        x_pp = (K_P * error) + (K_I * integral) + (K_D * derivative)

        # Save error value for next iteration
        error_prev = error

        #Update Graph
        t.append(monotonic_ns()*1e-9-START_TIME)
        Kalman_angles.append(KalmanAnglePitch)
        Gyro_angles.append(Gyro_angles[-1] + RatePitch*IT_time)
        Acc_angles.append(AnglePitch)
        Mod_Acc_angles.append(Mod_AnglePitch)
        Mod_Kalman_angles.append(Mod_KalmanAnglePitch)
        X_Acc_l.append(x_pp)
        IT_TIMES_l.append(IT_time)

        Iteration = Iteration+1

       # Too far to continue
        if KalmanAnglePitch > safe[1] or KalmanAnglePitch < safe[0]:
            print('It\'s over my friend :(')
            break

        # Problem 
        if IT_time>IT_TIME_TARGET*2:
            logger.fatal("Die, Iteration target time %s s, Step delay time %s s, Number of Step %s, Iteration time %s s, Step Mode %s" % (IT_TIME_TARGET, Step_Delay*1e-9, N_Step, IT_time, Step_Mode))
            break

# Scavenging work after the end of the program
except KeyboardInterrupt:
    GPIO.output(EN_PIN, GPIO.HIGH)

# Disable motors
GPIO.output(EN_PIN, GPIO.HIGH)

print('Stopped')

# Convert radians to degrees
Acc_angles = np.array(Acc_angles)*180/np.pi
Mod_Acc_angles = np.array(Mod_Acc_angles)*180/np.pi
Gyro_angles = np.array(Gyro_angles)*180/np.pi
Kalman_angles = np.array(Kalman_angles)*180/np.pi
Mod_Kalman_angles = np.array(Mod_Kalman_angles)*180/np.pi

# Store data
if Datasave == True:
    raw_data = np.column_stack((t, Acc_raw[0], Acc_raw[1], Gyro_raw, Acc_angles, Gyro_angles, Kalman_angles, X_Acc_l, IT_TIMES_l))
    np.savetxt('/home/arthur/TIPE/PID_calibration/K_P=%s,K_I=%s,K_D=%s.csv' % (K_P, K_I, K_D), raw_data, header="time, Acc_raw_x, Acc_raw_z, Gyro_raw, Acc_angles, Gyro_angles, Kalman_angles, x_pp, iteration time", delimiter=" ")

#plt.subplot(211)
plt.plot(t, Acc_angles, 'c--', label = "Acceleration calculated angle")
#plt.plot(t, Mod_Acc_angles, 'g--', label = "Moded Acceleration calculated angle")
plt.plot(t, Gyro_angles,  'm--', label = "Gyro")
plt.plot(t, Kalman_angles, 'k--', label = "Kalman")
#plt.plot(t, Mod_Kalman_angles, 'r--', label = "Mod_Kalman")
plt.xlabel("Temps (en seconde)")
plt.ylabel("Angles (en degres)")
plt.legend()
plt.grid()
#
#plt.subplot(212)
#plt.plot(t, Acc_raw[0], 'k--', label = 'Raw x Acceleration')
#plt.plot(t, Acc_raw[1], 'b--', label = 'Raw z Acceleration')
#plt.plot(t, Mod_Acc_raw[0], 'm--', label = 'Modded x Acceleration')
#plt.plot(t, Mod_Acc_raw[1], 'r--', label = 'Modded z Acceleration')
#plt.plot(t, X_Acc_l, 'g--', label = 'Acceleration')
#plt.plot(t, velocity_list, 'k')
plt.xlabel("Temps (en seconde)")
plt.ylabel("Accélération en m/s^2")
plt.legend()
plt.grid()

plt.savefig('/home/arthur/TIPE/PID_calibration/K_P = %s, K_I = %s, K_D = %s.png' % (K_P, K_I, K_D))
