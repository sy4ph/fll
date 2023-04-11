#!/usr/bin/env python3

from ev3dev2.motor import (
    OUTPUT_C,
    OUTPUT_D,
    OUTPUT_A,
    OUTPUT_B,
    MoveDifferential,
    SpeedRPM,
    MoveTank,
    SpeedPercent,
    follow_for_ms,
    MediumMotor,
    LargeMotor,
)
import threading
from ev3dev2.button import *
from ev3dev2.sensor.lego import GyroSensor
from ev3dev2.sensor import INPUT_4
from ev3dev2.wheel import Wheel
from ev3dev2.display import *
import time

STUD_MM = 8


class TechnicWheel(Wheel):
    def __init__(self):
        self.diameter_mm = 61.4
        self.width_mm = 20
        self.circumference_mm = 2 * 3.14159 * 31.2


gyro = GyroSensor(INPUT_4)
tank = MoveTank(OUTPUT_D, OUTPUT_C)
tank.gyro = gyro
mani_motor_1 = MediumMotor(OUTPUT_A)
mani_motor_2 = MediumMotor(OUTPUT_B)
move1 = LargeMotor(OUTPUT_D)
move2 = LargeMotor(OUTPUT_C)


def mani_1_on(speed, rotations):
    mani_motor_1.on_for_rotations(SpeedPercent(speed), rotations)


def mani_2_on(speed, rotations):
    mani_motor_2.on_for_rotations(SpeedPercent(-speed), rotations)


# multithreading for manipulator
def mani_on(rotations, speed=40):
    def mani_1_on():
        mani_motor_1.on_for_rotations(SpeedPercent(speed), rotations)

    def mani_2_on():
        mani_motor_2.on_for_rotations(SpeedPercent(-speed), rotations)

    thread1 = threading.Thread(target=mani_1_on)
    thread2 = threading.Thread(target=mani_2_on)
    thread1.start()
    thread2.start()
    thread1.join()
    thread2.join()
    mani_motor_1.off()
    mani_motor_2.off()


kp = 0.86
ki = 0.03
kd = 0.10


def MoveInAngleLine(speed, desired_angle, rotations_to):
    start_rot_1 = move1.rotations
    start_rot_2 = move2.rotations
    rotations_to1 = -rotations_to
    integral = 0
    last_error = 0
    error = 0
    print(max(start_rot_1, start_rot_2))
    math = max(move1.rotations, move2.rotations) - max(start_rot_1, start_rot_2)
    while math > rotations_to1 + 0.03 or math < rotations_to1 - 0.03:
        math = max(move1.rotations, move2.rotations) - max(start_rot_1, start_rot_2)
        error = desired_angle - gyro.angle
        integral = integral + error
        derivative = error - last_error
        turn = kp * error + ki * integral + kd * derivative
        left_speed = -speed + turn
        right_speed = -speed - turn
        if left_speed > 100:
            left_speed = 100
        if left_speed < -100:
            left_speed = -100
        if right_speed > 100:
            right_speed = 100
        if right_speed < -100:
            right_speed = -100
        tank.on(left_speed=left_speed, right_speed=right_speed)

        time.sleep(0.0001)
    # while move1.rotations > move2.rotations:
    #     tank.on(50, 0)
    # while move2.rotations > move1.rotations:
    #     tank.on(0, 50)
    tank.off()


def turn_left(degrees):
    while gyro.angle > degrees:
        tank.on(left_speed=-15, right_speed=15)
        time.sleep(0.001)
    tank.off()


def turn_right(degrees):
    while gyro.angle < degrees:
        tank.on(left_speed=15, right_speed=-15)
        time.sleep(0.001)
    tank.off()


# Calibrate the gyro sensor
def gyro_cal():
    gyro.calibrate()
    gyro.mode = "GYRO-CAL"
    gyro.mode = "GYRO-ANG"


gyro_cal()
mani_2_on(10, 0.8)
MoveInAngleLine(60, 0, 1)
MoveInAngleLine(15, 0, 0.7)
MoveInAngleLine(-60, 0, -0.5)
mani_2_on(-20, 1)
turn_left(-45)
gyro_cal()
MoveInAngleLine(50, 0, 2.68)
turn_right(90)
mani_1_on(20, 0.3)
MoveInAngleLine(30, 95, 0.91)
mani_1_on(-20, 1)
for i in range(4):
    MoveInAngleLine(-40, 95, -0.5)
    MoveInAngleLine(40, 95, 0.5)
