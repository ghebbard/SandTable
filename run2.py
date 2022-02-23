import RPi.GPIO as GPIO
from utils.DRV8825 import DRV8825
import threading
import math
from time import sleep
from random import shuffle
from subprocess import call

from utils.process_files import get_files, process_new_files, read_track, get_max_disp

# Motor driver object init
M_Rot = DRV8825(dir_pin=13, step_pin=19, enable_pin=12, mode_pins=(16, 17, 20))
M_Lin = DRV8825(dir_pin=24, step_pin=18, enable_pin=4, mode_pins=(21, 22, 27))

# Setting microstep size to 1/8
M_Rot.set_microstep('hardward','1/4step')
M_Lin.set_microstep('hardward','1/4step')


# Setup for limit switches
outer_switch = 5
inner_switch = 6

GPIO.setmode(GPIO.BCM)
GPIO.setup(outer_switch, GPIO.IN)
GPIO.setup(inner_switch, GPIO.IN)

# Slide thresholds
center_to_min = 250
outer_to_max = 250

# Functions defined for each motor thread
def run_MRot(steps, delay):
    if steps != 0 and delay >= 0:
        if steps > 0:
            M_Rot.turn_steps(Dir='forward', steps=abs(steps), stepdelay=delay)
        else:
            M_Rot.turn_steps(Dir='backward', steps=abs(steps), stepdelay=delay)

    M_Rot.stop()
    M_Rot.running = False

    # print("M_Rot done!")


def run_MRot_until(Dir, delay):
    if delay >= 0:
        while M_Rot.running:
            M_Rot.turn_steps(Dir=Dir, steps=1, stepdelay=delay)

    M_Rot.stop()

    # print("M_Rot done!")


def run_MLin(steps, delay):
    if steps != 0 and delay >= 0:
        if steps > 0:
            M_Lin.turn_steps(Dir='forward', steps=abs(steps), stepdelay=delay)
        else:
            M_Lin.turn_steps(Dir='backward', steps=abs(steps), stepdelay=delay)

    M_Lin.stop()
    M_Lin.running = False

    # print("M_Lin done!")


def run_MLin_until(steps, delay):
    run_MLin(steps, delay)
    M_Rot.running = False


# Calibrates the linear slide arm before starting the main program routine
def calibrate_slide():

    calibrated = False

    while not calibrated:
        minPos = M_Lin.turn_until_switch(Dir='backward', limit_switch=inner_switch, stepdelay=0.0002)
        maxPos = M_Lin.turn_until_switch(Dir='forward', limit_switch=outer_switch, stepdelay=0.0002) + minPos

        print((minPos, maxPos))
        totalDist = maxPos - minPos - center_to_min - outer_to_max
        print ("Travel distance: " + str(totalDist))

        sleep(.5)
        test_inner = M_Lin.turn_check_cali(Dir='backward', steps=totalDist + outer_to_max, limit_switch=inner_switch, stepdelay=0.0002)
        # sleep(.5)
        # test_outer = M_Lin.turn_check_cali(Dir='forward', steps=totalDist, limit_switch=outer_switch, stepdelay=0.0002)

        if test_inner:
            calibrated = True
            print("Calibration Passed!")
            # sleep(.5)
            # M_Lin.turn_steps(Dir='backward', steps=totalDist, stepdelay=0.0002)
        else:
            print("Calibration Failed! Trying again...")

    return totalDist


def erase_out_to_in():

    M_Rot.running = True
    M_Lin.running = True

    sleep(1)
    M_Lin.turn_until_switch(Dir='forward', limit_switch=outer_switch, stepdelay=0.0002)
    M_Lin.turn_steps(Dir='backward', steps=outer_to_max, stepdelay=0.0002)
    print("Found edge")

    sleep(.5)
    MRot = threading.Thread(target=run_MRot_until, args=('forward', 0.0005,))
    MLin = threading.Thread(target=run_MLin_until, args=(-max_disp, 0.01,))

    print("Erasing...")
    MRot.start()
    MLin.start()

    MRot.join()
    MLin.join()


# def erase_in_to_out():
#     sleep(1)
#     M_Lin.turn_until_switch(Dir='backward', limit_switch=inner_switch, stepdelay=0.0002)
#     M_Lin.turn_steps(Dir='forward', steps=center_to_min, stepdelay=0.0002)
#     print("Found edge")

#     sleep(.5)
#     MRot = threading.Thread(target=run_MRot_until, args=('forward', 0.00035,))
#     MLin = threading.Thread(target=run_MLin_until, args=(max_disp, 0.01,))

#     print("Erasing...")
#     MRot.start()
#     MLin.start()

#     MRot.join()
#     MLin.join()


class InterfaceThread():

    def __init__(self):
        self.running = True
        self.collision_start_time = None
        self.main_start_time = None
        self.limit_pressed = False
        self.collision_detected = False
        self.stop_program = False
        self.erase = False
        self.ask_erase = False
        self.currently_displayed = []

    def check_collision(self):
        if GPIO.input(inner_switch) == 0 or GPIO.input(outer_switch) == 0:
            if not self.limit_pressed:
                self.collision_start_time = int(round(time.time() * 1000))
                self.limit_pressed = True

            if self.limit_pressed and self.collision_start_time != None:
                if int(round(time.time() * 1000)) - self.collision_start_time > 2000:
                    print("\n---------- Collision Detected! ----------")
                    stop_motors()
                    self.collision_detected = True
        else:
            self.limit_pressed = False


# Stops the motors and LED strip, and joins the threads
def stop_program(shutdown=False):
    stop_motors()

    if shutdown:
        sleep(2)

        GPIO.cleanup()

        call("sudo shutdown -h now", shell=True)
    else:
        GPIO.cleanup()
        print("Exiting...")
        exit()


def stop_motors():
    M_Rot.running = False
    M_Lin.running = False
    M_Rot.stop()
    M_Lin.stop()
    print("\n---------- Motors Stopped! ----------")


# Switches thread object init
interface = InterfaceThread()

def main():
    global max_disp

    max_disp = calibrate_slide()
    try:
        erase_out_to_in()

        if interface.stop_program:
            stop_program(shutdown=True)

    except KeyboardInterrupt:
        stop_program()