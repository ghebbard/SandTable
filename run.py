import RPi.GPIO as GPIO
from utils.DRV8825 import DRV8825
import threading
import math
from time import sleep
from random import shuffle
from subprocess import call

from rpi_ws281x import PixelStrip, Color
from led_strip import *

from utils.process_files import get_files, process_new_files, read_track, get_max_disp

# Motor driver object init
M_Rot = DRV8825(dir_pin=13, step_pin=19, enable_pin=12, mode_pins=(16, 17, 20))
M_Lin = DRV8825(dir_pin=24, step_pin=18, enable_pin=4, mode_pins=(21, 22, 27))

# Setting microstep size to 1/8
M_Rot.set_microstep('software','1/4step')
M_Lin.set_microstep('software','1/4step')

# Create NeoPixel object with appropriate configuration.
strip = strip_init()
strip_thread = LedStripThread()

# Setup for limit switches
outer_switch = 5
inner_switch = 6
#motor_relay = 23
#led_relay = 25
#main_button = 26

GPIO.setmode(GPIO.BCM)
GPIO.setup(outer_switch, GPIO.IN)
GPIO.setup(inner_switch, GPIO.IN)
#GPIO.setup(main_button, GPIO.IN)
#GPIO.setup(motor_relay, GPIO.OUT)
#GPIO.setup(led_relay, GPIO.OUT)

# Slide thresholds
center_to_min = 250
outer_to_max = 250


# Run through the LED strip routine
def run_LedStrip():
    strip.begin()

    while strip_thread.running:
        print('LED Color wipe')
        strip_thread.colorWipe(strip, Color(255, 0, 0))  # Red wipe
        strip_thread.colorWipe(strip, Color(0, 255, 0))  # Blue wipe
        strip_thread.colorWipe(strip, Color(0, 0, 255))  # Green wipe
        print('LED Theater chase')
        if not strip_thread.running: return
        strip_thread.theaterChase(strip, Color(127, 127, 127))  # White theater chase
        strip_thread.theaterChase(strip, Color(127, 0, 0))  # Red theater chase
        strip_thread.theaterChase(strip, Color(0, 0, 127))  # Blue theater chase
        print('LED Rainbow animations')
        if not strip_thread.running: return
        strip_thread.rainbow(strip)
        strip_thread.rainbowCycle(strip)
        strip_thread.theaterChaseRainbow(strip)


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
        #self.main_pressed = False
        self.collision_detected = False
        self.stop_program = False
        self.displaying_options = False
        self.next_drawing = False
        self.erase = False
        self.ask_erase = False

        self.options = {0: "Back", 1: "Shutdown", 2: "Stop/erase"}
        self.selected_option = 0

        self.currently_displayed = []

    # def check_all_switches(self):
    #     while self.running:
    #         sleep(.1)
    #
    #         if not self.ask_erase:
    #             if not self.main_pressed and GPIO.input(main_button) == 1:
    #                 self.main_pressed = True
    #                 self.main_start_time = int(round(time.time() * 1000))
    #
    #             if self.main_pressed:
    #                 # if GPIO.input(main_button) == 1 and int(round(time.time() * 1000)) - self.main_start_time > 3000:
    #                 #     self.main_pressed = False
    #                 #     self.stop_program = True
    #                 #     self.running = False
    #                 #     print("Shutdown!")
    #                 #     # stop_motors()
    #
    #                 if GPIO.input(main_button) == 0:
    #                     self.main_pressed = False
    #
    #             if self.main_pressed:
    #                 if GPIO.input(main_button) == 0 and int(round(time.time() * 1000)) - self.main_start_time > 1000:
    #                     self.main_pressed = False
    #                     self.select_option()
    #                 elif GPIO.input(main_button) == 0:
    #                     self.main_pressed = False
    #                     self.selected_option = (self.selected_option + 1) % 3

    def select_option(self):
        if self.selected_option == 0:
            print("Back")

        elif self.selected_option == 1:
            self.stop_program = True
            self.running = False
            print("Shutdown!")
            stop_motors()
        else:
            self.next_drawing = True
            print("Erasing!")
            stop_motors()

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

    strip_thread.running = False
    strip_thread.colorWipe(strip, Color(0, 0, 0))
    LStrip.join()

    interface.running = False
    #interface_thread.join()

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

# Create switches thread
#interface_thread = threading.Thread(target=interface.check_all_switches)

# Create LStrip thread
LStrip = threading.Thread(target=run_LedStrip)

def main():
    global max_disp

    try:
        #GPIO.output(motor_relay, GPIO.LOW)
        #GPIO.output(led_relay, GPIO.LOW)

        LStrip.start()

        process_new_files(Dir="/home/pi/Sand-Table/")

        # files = get_files(Dir="/home/pi/Sand-Table/")
        with open("/home/pi/Sand-Table/filenames.txt", "r") as f:
            content = f.readlines()
        files = [line.rstrip('\n') for line in content]
        shuffle(files)

        #interface_thread.start()

        max_disp = calibrate_slide()

        first_file = True

        while not interface.stop_program:

            for f in files:
                if interface.stop_program:
                    break

                if not first_file:
                    if not interface.erase and not interface.next_drawing:
                        continue
                    else:
                        interface.next_drawing = False
                        interface.erase = False

                    erase_out_to_in()

                    if interface.stop_program:
                        break

                print("Running: {}".format(f))

                track = read_track(f, Dir="/home/pi/Sand-Table/")

                # prog_disp_interrupted = False

                first_step = True

                for i, step in enumerate(track):
                    print(step)

                    # Create motor threads
                    MRot = threading.Thread(target=run_MRot, args=(step[0], step[2],))
                    MLin = threading.Thread(target=run_MLin, args=(step[1], step[3],))

                    print("...")
                    M_Rot.running = True
                    M_Lin.running = True
                    MRot.start()
                    MLin.start()

                    MRot.join()
                    MLin.join()

                    if interface.collision_detected:
                        interface.limit_pressed = False
                        break

                    if interface.stop_program or interface.next_drawing:
                        break

                    # print("Motors done!")
                    first_step = False

                first_file = False

        if interface.stop_program:
            stop_program(shutdown=True)

    except KeyboardInterrupt:
        stop_program()


if __name__ == '__main__':
    main()
