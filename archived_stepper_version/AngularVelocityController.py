#!/usr/bin/env python

import numpy as np
from time import time
from time import sleep
from time import monotonic_ns
import sys
#from RpiMotorLib import RpiMotorLib
import RPi.GPIO as GPIO
import asyncio
import logging

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO, format='%(asctime)s %(message)s') # logging.INFO, logging.DEBUG

#From RpiMotorLib
class StopMotorInterrupt(Exception):
    """ Stop the motor """
    pass

    #Me

    #Me
def ns_sleep(wait_time):
    ''' sleep for a set time in nanoseconds'''
    start_time = monotonic_ns()
    cur_time = monotonic_ns()
    while cur_time - start_time <= wait_time:
        cur_time = monotonic_ns()

class Angular_Velocity_Controller:
    def __init__(self, Step_Modes_Min_Delay, DIRECTION, STEP, GPIO_Pins=(-1,-1,-1), Motor_Type = "DRV8825"):
        ''' (1) Step modes minimal delay in microseconds
            (2) gpio port for direction
            (3) gpio port for STEP
            (4) gpio pins to control mode (-1,-1,-1) if not necessary'''
        self.direction_pin = DIRECTION
        self.step_pin = STEP
        self.motor_type = Motor_Type
        self.mode_pins = GPIO_Pins
        self.micro_sec = 1e-6
        self.execution_time = 4 # time to execute a command in microseconde
        if self.motor_type == "DRV8825":
            self.STEP_ANGLE = 1.8*np.pi/180 #In radians
            self.STEP_MODES_STR = ["Full", "Half", "1/4", "1/8", "1/16", "1/32"]
            self.STEP_MODES_FLOAT = np.array([1/(2**i) for i in range(6)])
        else:
            print('Not available')
        self.STEP_MODES_Min_Delay = 1e3*Step_Modes_Min_Delay # np.array([1300, 700, 500, 300, 160, 80]) Switched in nanoseconds

    #Me
    def STEP_possibility(self, velocity, Alloted_Time):
        ''' Return list of possible step_delays and numbers of steps needed
         (1) (positive) velocity in radians per seconds
         (2) Allotted time in seconds
        '''
        step_delays = (self.STEP_ANGLE * self.STEP_MODES_FLOAT/velocity) # Array of all possible waited time between steps, in seconds
        steps = Alloted_Time/step_delays # Array of numbers of steps taken, in order of the step modes.
        step_delays = step_delays*1e9 # In nanoseconds
        return(step_delays, steps)

    #Me
    def optimal_STEP(self, velocity, Alloted_Time, Step_Mode):
        ''' Returns the optimal step mode and step delay for a given (positive) velocity (in radians per seconds) and the allotted time (in seconds).
        Returns -1 if none are possible'''
        step_delays, steps = self.STEP_possibility(velocity, Alloted_Time)
        STEP_Difference = step_delays - self.STEP_MODES_Min_Delay 

        # Set Default optimal values
        Optimal_Step_Delay = -1
        Optimal_Step_Mode = -1
        Optimal_Step = -1

        # Set the Step mode if forced
        if Step_Mode != 0:
            ind = self.STEP_MODES_STR.index(Step_Mode)
            Optimal_Step_Delay = step_delays[ind]
            Optimal_Step_Mode = self.STEP_MODES_STR[ind]
            Optimal_Step = steps[ind]
        elif velocity*Alloted_Time > self.STEP_ANGLE*self.STEP_MODES_FLOAT[-1]: # Check if angle is possible with highest resolution
            # Browse different options to find the best one
            for i in range(len(self.STEP_MODES_STR)):
                # The arrays are browsed backward because the smallest step_mode is prefered as it is smoother.
                if STEP_Difference[-(i+1)] > 0:
                    Optimal_Step_Delay = step_delays[-(i+1)]
                    Optimal_Step_Mode = self.STEP_MODES_STR[-(i+1)]
                    Optimal_Step = steps[-(i+1)]
                    break
                elif i == len(self.STEP_MODES_STR):
                    logger.info("Not possible, cause: velocity asked to high")
        else:
            logger.info("Not possible, cause: angle required lower than highest resolution")
        return Optimal_Step_Mode, Optimal_Step_Delay, Optimal_Step

    # From RpiMotorLib
    def motor_stop(self):
        """ Stop the motor """
        self.stop_motor = True

    def resolution_set(self, steptype):
        """ method to calculate step resolution
        based on motor type and steptype"""
        if self.motor_type == "DRV8825":
            resolution = {'Full': (0, 0, 0),
                          'Half': (1, 0, 0),
                          '1/4': (0, 1, 0),
                          '1/8': (1, 1, 0),
                          '1/16': (0, 0, 1),
                          '1/32': (1, 0, 1)}
        else:
            print(" Not available")
            quit()

        # error check stepmode
        if steptype in resolution:
            pass
        else:
            print("Error invalid steptype: {}".format(steptype))
            quit()

        if self.mode_pins != False:
            GPIO.output(self.mode_pins, resolution[steptype])

    #Modified version of Rpi.Motorlib
    def non_linear_timed_motor_go(self, clockwise=False, steptype="Full", Alloted_time = 5, stepdelay=150000, initdelay=.00, steps = 200):
        # The problem with the current program that ensures that the total angle is respected is that the speed isn't constant to catch up the delay.
        """ motor_go,  moves stepper motor based on 6 inputs

         (1) clockwise, type=bool default=False
         help="Turn stepper counterclockwise"
         (2) steptype, type=string , default=Full help= type of drive to
         step motor 5 options
            (Full, Half, 1/4, 1/8, 1/16) 1/32 for DRV8825 only 1/64 1/128 for LV8729 only
         (3) Alloted time, type = int, default=5, in seconds
         (4) stepdelay, type=float, default=0.05, help=Time to wait
         (in nanoseconds) between steps.
         (6) initdelay, type=float, default=1mS, help= Intial delay after
         GPIO pins initialized but before motor is moved.
         (7) number of total steps, type = int
        """

        self.stop_motor = False
        # setup GPIO
        GPIO.setup(self.direction_pin, GPIO.OUT)
        GPIO.setup(self.step_pin, GPIO.OUT)
        GPIO.output(self.direction_pin, clockwise)
        if self.mode_pins != False:
            GPIO.setup(self.mode_pins, GPIO.OUT)
    
        try:
            # dict resolution
            self.resolution_set(steptype)
            sleep(initdelay*self.micro_sec)

            #l = []
            #l2 = []
            l3 = []
            START = monotonic_ns()
            Corrected_stepdelay = stepdelay
            step_counter = 0
            Alloted_time = Alloted_time*1e9
            Time_Passed = monotonic_ns() - START
            Time_left = Alloted_time
            while Time_Passed < Alloted_time:  # convert stepdelay from seconds to nanoseconds for monotonic_ns()
                #start = monotonic_ns()

                GPIO.output(self.step_pin, True)
                ns_sleep(Corrected_stepdelay/2)
                GPIO.output(self.step_pin, False)
                ns_sleep(Corrected_stepdelay/2)

                step_counter+=1

                Time_Passed = monotonic_ns() - START
                Time_left = Alloted_time - (Time_Passed)
                if steps != step_counter: # No need to calculate when all steps have been done
                    Corrected_stepdelay = Time_left/(steps - step_counter) 
            #print("Number of steps taken", step_counter, "stepdelay", stepdelay)



        except KeyboardInterrupt:
            print("User Keyboard Interrupt : RpiMotorLib:")
        except StopMotorInterrupt:
            print("Stop Motor Interrupt : RpiMotorLib: ")
        except Exception as motor_error:
            print(sys.exc_info()[0])
            print(motor_error)
            print("RpiMotorLib  : Unexpected error:")

        finally:
            # cleanup
            GPIO.output(self.step_pin, False)
            GPIO.output(self.direction_pin, False)
            if self.mode_pins != False:
                for pin in self.mode_pins:
                    GPIO.output(pin, False)

    #Modified version of Rpi.Motorlib
    def timed_motor_go(self, clockwise=False, steptype="Full", Alloted_time = 5, stepdelay=150000, initdelay=.00, steps = 200):
        """ motor_go,  moves stepper motor based on 6 inputs

         (1) clockwise, type=bool default=False
         help="Turn stepper counterclockwise"
         (2) steptype, type=string , default=Full help= type of drive to
         step motor 5 options
            (Full, Half, 1/4, 1/8, 1/16) 1/32 for DRV8825 only 1/64 1/128 for LV8729 only
         (3) Alloted time, type = int, default=5, in seconds
         (4) stepdelay, type=float, default=0.05, help=Time to wait
         (in nanoseconds) between steps.
         (6) initdelay, type=float, default=1mS, help= Intial delay after
         GPIO pins initialized but before motor is moved.
         (7) number of total steps, type = int
        """

        self.stop_motor = False
        # setup GPIO
        GPIO.setup(self.direction_pin, GPIO.OUT)
        GPIO.setup(self.step_pin, GPIO.OUT)
        GPIO.output(self.direction_pin, clockwise)
        if self.mode_pins != False:
            GPIO.setup(self.mode_pins, GPIO.OUT)
    
        try:
            # dict resolution
            self.resolution_set(steptype)
            sleep(initdelay*self.micro_sec)

            #l = []
            #l2 = []
            l3 = []
            START = monotonic_ns()
            Loop_End_Time = START
            Corrected_stepdelay = stepdelay
            Difference_Reality_Goal = 0
            step_counter = 0
            #for _ in range(steps):
            while Loop_End_Time-START < Alloted_time*1e9:  # convert stepdelay from seconds to nanoseconds for monotonic_ns()
                #start = monotonic_ns()

                Start_Step_Time = monotonic_ns()
                GPIO.output(self.step_pin, True)
                ns_sleep(Corrected_stepdelay/2)
                GPIO.output(self.step_pin, False)
                ns_sleep(Corrected_stepdelay/2)

                #analysis
                l3.append(Corrected_stepdelay)
                #l2.append(Corrected_stepdelay)
                #l.append(monotonic_ns()-start)
                step_counter+=1

                Loop_End_Time = monotonic_ns()
                Difference_Reality_Goal = ((Loop_End_Time-Start_Step_Time) - Corrected_stepdelay)
                Corrected_stepdelay = stepdelay - Difference_Reality_Goal
            #print("Waited time:", 1e-3*sum(l)/len(l), "Expected waited time:", stepdelay, "Number of steps taken", step, "Average Reality Goal difference", sum(l3)/len(l3))
            #print("Number of steps taken", step_counter, "Average corrected_stepdelay", sum(l3)/len(l3))


        except KeyboardInterrupt:
            print("User Keyboard Interrupt : RpiMotorLib:")
        except StopMotorInterrupt:
            print("Stop Motor Interrupt : RpiMotorLib: ")
        except Exception as motor_error:
            print(sys.exc_info()[0])
            print(motor_error)
            print("RpiMotorLib  : Unexpected error:")

        finally:
            # cleanup
            GPIO.output(self.step_pin, False)
            GPIO.output(self.direction_pin, False)
            if self.mode_pins != False:
                for pin in self.mode_pins:
                    GPIO.output(pin, False)

    #Me  
    def Vel_Time_Motor_Go(self, velocity, Alloted_Time, Step_Mode = 0):
        ''' Turns the motor (from RpiMotorLib)
         (1) velocity in radians per seconds
         (2) allotted time in seconds
        '''

        Turning_direction = True # True for clockwise and False for counter-clockwise
        if velocity <= 0:
            velocity = -velocity
            Turning_direction = False

        Opt_Step_Mode, Opt_Step_Delay, Opt_Step = self.optimal_STEP(velocity, Alloted_Time, Step_Mode)

        if (Opt_Step_Mode, Opt_Step_Delay, Opt_Step) != (-1,-1,-1):
            self.timed_motor_go(Turning_direction, Opt_Step_Mode , Alloted_Time, Opt_Step_Delay, 0, int(Opt_Step))
            logger.debug("Step mode: %s, Awaited number of steps: %s, Awaited step delay: %s , Normal total time: %s" % (Opt_Step_Mode, Opt_Step, Opt_Step_Delay, Opt_Step*Opt_Step_Delay))
            return Opt_Step_Mode, Opt_Step_Delay, Opt_Step
        else:
            ns_sleep(Alloted_Time*1e9)
            print("Not possible :(")
            return -1, -1, -1

    # Me
    def Angle_Motor_Go(self, angle, Alloted_time, Step_Mode):
        ''' Turns the motor (from RpiMotorLib)
         (1) Angle in radians
         (2) Allotted time in seconds
         (3) Step mode needed
        '''
        Turning_direction = True # True for clockwise and False for counter-clockwise
        if angle <= 0:
            angle = -angle
            Turning_direction = False
        
        ind = self.STEP_MODES_STR.index(Step_Mode)
        N_step = angle/(self.STEP_ANGLE*self.STEP_MODES_FLOAT[ind])
        Step_Delay = Alloted_time/N_step

        # Move Motor
        self.timed_motor_go(Turning_direction, Step_Mode, Alloted_time, Step_Delay*1e9, 0, int(N_step))


def test():
    GPIO_pins = (16, 20, 21) # Microstep Resolution MS0-MS2 (-1,-1,-1) if not used
    DIRECTION = 27 # Direction (DIR) GPIO Pin
    STEP = 17 # Step GPIO Pin
    EN_PIN = 23 # enable pin (LOW to enable)
    Step_Modes_Min_Delay = np.array([1300, 700, 500, 300, 160, 80]) #In micro seconds
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(EN_PIN,GPIO.OUT) # set enable pin as output
    Motor = Angular_Velocity_Controller(Step_Modes_Min_Delay, DIRECTION, STEP, GPIO_pins)
    GPIO.output(EN_PIN,GPIO.LOW)
    START = time()
    Motor.Vel_Time_Motor_Go(3.5*2*np.pi, 1)
    print(time()-START)
    GPIO.output(EN_PIN,GPIO.HIGH)
    