from time import sleep
from time import time
from time import monotonic_ns
import datetime
import numpy as np
import logging
# For motors
import RPi.GPIO as GPIO
from Motor_Control import DC_Motor_Controller
# For inertial measurement unit
import adafruit_mpu6050
import board
import busio

# For ps4 controller
from pyPS4Controller.controller import Controller

# To update PID with controller input
import threading

# Set debugger
logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO, format='%(asctime)s %(message)s') # logging.info, logging.debug

# Remove warnings
GPIO.setwarnings(False)

# Set board mode
GPIO.setmode(GPIO.BCM)

# Setup inertial measurement unit
i2c = busio.I2C(board.SCL, board.SDA)
mpu = adafruit_mpu6050.MPU6050(i2c)

# Correct Gyro error
Gryo_Calibration = 0.018219505384692924

# Correct inertial unit placement
Angle_Calibration = -0.04025219959708072
## Motor setup
# Voltage when the duty cycle is at 100
Vmax = 6.00

# Motor1
R_Motor = DC_Motor_Controller(in1=20, in2=16, en=21,Vmax=Vmax)

# Motor 2
L_Motor = DC_Motor_Controller(in1=12,in2=8, en=25, Vmax=Vmax)

## Encoder setup
# Number of ticks
ticks = 0

## Set initial data
# To have a first angle
acc_x, acc_y, acc_z = mpu.acceleration
Angle_Pitch = np.atan2(-acc_x, -acc_z)
Comp_angle = Angle_Pitch - Angle_Calibration

# Set initial value
Voltage_Command = 0
Last_angle_call = monotonic_ns()

# Set target angle
TARGET = 0 # In radians
safe = [-180*np.pi/180,180*np.pi/180]

# Number of times the angle is calculated per iteration
N_angle_call = 1

# Set target iteration time
IT_TIME_TARGET = 0.015

# Set PID
error = 0
integral = 0
derivative = 0
error_prev = 0

# To stay upright
K_P_stable = 23
K_I_stable = 105
K_D_stable  = 0.67
N_PID = 2120

# System is initially turned off
K_P = 0
K_I = 0
K_D = 0

# Store data, angles are in radians, the acceleration in m.s^-2
Comp_angle_L = [Angle_Pitch-Angle_Calibration]
Gyro_raw_L = [0]
Acc_raw_x_L = [acc_x]
Acc_raw_z_L = [acc_z]
time_L = [0]
start_time = datetime.datetime.now()
Program_start = time()

# Complementary angle definition
Gyro_weight = 0.9

def Comp_Angle_calc(Angle, Gyro, IT_time, Acc_angle):
   return Gyro_weight*(Angle+Gyro*IT_time) + (1-Gyro_weight)*Acc_angle

def Angle_Calc():
   global Comp_angle_L, Gyro_raw_L, Acc_raw_x_L, Acc_raw_z_L, Comp_angle, Last_angle_call

   # Time since last call
   iteration_time = (monotonic_ns() - Last_angle_call)*1e-9
   Last_angle_call = monotonic_ns()

   # Read accelerometer raw values
   acc_x, acc_y, acc_z = mpu.acceleration

   # Read gyroscope values
   gyro_y = -(mpu.gyro[1] - Gryo_Calibration)

   # Angle from accelerometer
   Angle_Pitch = np.atan2(-acc_x, -acc_z) - Angle_Calibration

   # Calculate angle with complementary filter
   Comp_angle = Comp_Angle_calc(Comp_angle, gyro_y, iteration_time, Angle_Pitch)

   # Uptade lists
   Comp_angle_L.append(Comp_angle)
   Gyro_raw_L.append(gyro_y)
   Acc_raw_x_L.append(acc_x)
   Acc_raw_z_L.append(acc_z)
   time_L.append(time()-Program_start)

def Set_Speed_W_Voltage_Get_Angle(voltage, Follow_time):
   global Comp_angle
   '''Set a voltage for both motors for a set amount of time
      (1) Voltage given to both motor
      (2) Time during which the motors spin'''
   # Turn on motors
   L_Motor.Command_by_Voltage(voltage)
   R_Motor.Command_by_Voltage(voltage)

   # Wait time in between angle calculation to create a average angle
   Wait = Follow_time/N_angle_call
   av_angle = 0

   Start_T = monotonic_ns()
   for _ in range(N_angle_call-1):
       Start_small_IT = monotonic_ns()
       Angle_Calc()
       av_angle += Comp_angle
       while monotonic_ns() - Start_small_IT < Wait*1e9:
           sleep(Wait/50)

   angle_start = monotonic_ns()
   Angle_Calc()
   angle_calc_L.append(monotonic_ns()-angle_start)

   av_angle += Comp_angle

   # Wait for the Follow time to have passed
   while monotonic_ns() - Start_T < Follow_time*1e9:
       sleep(Wait/50)

   Comp_angle = av_angle/N_angle_call

# Create a list to know the average time that Angle_Calc() takes
angle_calc_L = []

# Making sure that if there is a problem in setting up the motors, the motors aren't running
R_Motor.Motor_off()
L_Motor.Motor_off()

# Number of iterations
Iteration = 0

def correcteur(KI, KP, KD, Te):
   global error_l, Voltage_l
   A = np.array([KI*N_PID*Te**2 + (KP*N_PID+KI)*Te+KD*N_PID+KP,
                 -1*(KP*N_PID+KI)*Te - 2*(KD*N_PID+KP),
                 KD*N_PID+KP])
   Voltage_l = np.array([(Voltage_l[0]*(N_PID*Te+2)-Voltage_l[1]+np.dot(error_l,A))/(N_PID*Te+2),
                 Voltage_l[0],
                 Voltage_l[1]])

# Setup and Connect to PS4 Controller
class MyController(Controller):
   def __init__(self, **kwargs):
       Controller.__init__(self, **kwargs)

   def on_x_press(self):
       global TARGET, error, integral, derivative, error_prev, K_P, K_I, K_D
       print('PID for upright')
       TARGET = 0
       error, integral, derivative, error_prev = 0,0,0,0
       K_P, K_I, K_D = K_P_stable, K_I_stable, K_D_stable

   def on_circle_press(self):
       global error, integral, derivative, error_prev, K_P, K_I, K_D
       print('PID off')
       K_P, K_I, K_D = 0,0,0

   def on_R1_press(self):
       global TARGET,  error, integral, derivative, error_prev, K_P, K_I, K_D
       print('PID for forward')
       error, integral, derivative, error_prev = 0,0,0,0
       TARGET = 2*np.pi/180 #Target of 2 degrees to go forward
       K_P, K_I, K_D = K_P_stable, K_I_stable, K_D_stable

   def on_L1_press(self):
       global TARGET, error, integral, derivative, error_prev, K_P, K_I, K_D
       error, integral, derivative, error_prev = 0,0,0,0
       TARGET = -2*np.pi/180 #Target of 2 degrees to go forward
       K_P, K_I, K_D = K_P_stable, K_I_stable, K_D_stable

# Create and start the controller thread
controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)

def controller_thread():
   controller.listen(timeout=5)

# Start the controller in a separate thread
threading.Thread(target=controller_thread, daemon=True).start()
print('Controller thread started')

IT_real_time_L = []
try: # Main program loop
   while Iteration < 10000:
       logger.debug("Starting iteration %s" % Iteration)
       IT_START = monotonic_ns()*1e-9 # In seconds

       logger.debug(f"Activating motors to move at {Voltage_Command} Volt")
       Set_Speed_W_Voltage_Get_Angle(Voltage_Command, IT_TIME_TARGET)

       #Iteration Time
       Real_IT_time = monotonic_ns()*1e-9-IT_START
       logger.debug(f"Iteration time {Real_IT_time}")
       logger.debug(f"Average angle over 3 measurement {Comp_angle}")
       logger.debug(f"Calculating system angle in radians: Angle_Pitch = {Comp_angle_L[-1]}, RatePitch = {Gyro_raw_L[-1]}")
       error_l = np.array([- TARGET + Comp_angle, error_l[0], error_l[1]])
       correcteur(K_I, K_P, K_D, Real_IT_time)

       Iteration = Iteration+1

       # Too far to continue
       if Comp_angle > safe[1] or Comp_angle < safe[0]:
           print('It\'s over my friend :(')
           break

       IT_real_time_L.append(Real_IT_time)
       # Problem
       if Real_IT_time>IT_TIME_TARGET*2:
           logger.debug(f"Die, Iteration target time {IT_TIME_TARGET} , Iteration time {Real_IT_time}")
           #break
# Scavenging work after the end of the program
except KeyboardInterrupt:
   pass
# Disable motors
R_Motor.Motor_off()
L_Motor.Motor_off()

print('Stopped')

print(sum(angle_calc_L)/len(angle_calc_L)*1e-9)
print(f'Average iteration time: {sum(IT_real_time_L)/len(IT_real_time_L)}, when target is {IT_TIME_TARGET}')

# Save data
raw_data = np.column_stack((time_L, Acc_raw_x_L, Acc_raw_z_L, Gyro_raw_L, Comp_angle_L))
np.savetxt('Evolutiondeangle_Mot.csv',
      raw_data,
      header=f"time={start_time}, acc_x, acc_z, gyro, complemantary_angle",
      delimiter=" ")
