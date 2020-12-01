from spike import (
    PrimeHub,
    LightMatrix,
    Button,
    StatusLight,
    ForceSensor,
    MotionSensor,
    Speaker,
    ColorSensor,
    App,
    DistanceSensor,
    Motor,
    MotorPair,
)
from spike.control import wait_for_seconds, wait_until, Timer
import time
from math import pi
import random


class Robot:
    def __init__(self):
        self.hub = PrimeHub()
        self.wheels = MotorPair("A", "B")
        self.left_wheel = Motor("A")
        self.right_wheel = Motor("B")
        self.color_sensor = ColorSensor("E")
        self.motion_sensor = self.hub.motion_sensor
        self.arm = Motor("C")
        self.cube_dropper = Motor("D")

    def reset_distance_travelled(self):
        self.left_wheel.set_degrees_counted(degrees_counted=0)
        self.right_wheel.set_degrees_counted(degrees_counted=0)

    def distance_travelled(self):
        Turning = self.left_wheel.get_degrees_counted()
        Cards = self.right_wheel.get_degrees_counted()
        average = (-1 * Turning + Cards) / 2 / 360
        distance = average * 9 * pi
        return distance

    def follow_line(self, distance, speed):
        self.reset_distance_travelled()
        while self.distance_travelled() < distance:
            fred = self.color_sensor.get_reflected_light() - 60
            fred = fred * 1.2
            fred = round(fred)
            self.wheels.start(steering=fred, speed=speed)
        self.wheels.stop()

    def turn_to_direction(self, degree):
        while self.motion_sensor.get_yaw_angle() % 360 != degree:
            T = self.motion_sensor.get_yaw_angle() - degree
            T = T % 360
            if T < 180:
                steering = -100
            else:
                steering = 100
            self.wheels.start(steering=steering, speed=10)
        self.wheels.stop()

    def drive_in_direction(self, direction, distance, speed):
        self.reset_distance_travelled()
        while True:
            if distance < 0 and self.distance_travelled() < distance:
                break
            if distance > 0 and self.distance_travelled() > distance:
                break
            T = self.motion_sensor.get_yaw_angle() % 360 - direction
            T = T % 360
            if T < 180:
                steering = 0 - T
            else:
                steering = 360 - T
            if speed < 0:
                steering = -1 * steering
            self.wheels.start(steering=steering * 3, speed=speed)
        self.wheels.stop()

    def drop_block(self):
        self.cube_dropper.run_for_rotations(rotations=1, speed=100)

    def set_arm_height(self, degrees, speed=None):
        if speed is None:
            speed = 30
        if degrees < 0:
            degrees = 0
        if degrees > 300:
            degrees = 300
        current = self.arm.get_position()
        if degrees > current or current > 350:
            direction = "clockwise"
        else:
            direction = "counterclockwise"
        self.arm.run_to_position(degrees, direction, speed)


def test():
    robot = Robot()
    robot.motion_sensor.reset_yaw_angle()

    robot.drive_in_direction(0, 78, 20)
    robot.turn_to_direction(45)
    robot.drive_in_direction(50, -5, -20)
    robot.wheels.start(0, 20)
    while robot.color_sensor.get_color() != "black":
        pass
    robot.wheels.stop()
    robot.wheels.start(0, -10)
    while robot.color_sensor.get_color() != "white":
        pass
    robot.wheels.stop()
    robot.turn_to_direction(2)
    robot.drive_in_direction(2, 27, 20)
    robot.right_wheel.run_for_rotations(4, 20)
    robot.drive_in_direction(0, -10, -20)


def reset():
    robot = Robot()
    robot.set_arm_height(0)


def run_one():
    robot = Robot()
    robot.motion_sensor.reset_yaw_angle()
    # robot.drive_in_direction( 0, 20, 20)
    robot.set_arm_height(300)
    robot.drive_in_direction(0, 81, 20)
    # robot.drive_in_direction(0, 22, 3)
    # for i in range(5):

    #     robot.drive_in_direction(0, 4, 3)
    #     robot.drive_in_direction(0, -0.1, -20)
    for i in range(10):
        robot.wheels.start(0, 3)
        time.sleep(3)
        robot.wheels.start(0, -10)
        time.sleep(0.4)

    robot.drive_in_direction(0, -100, -30)

    # robot.wheels.start(0, -20)
    # while robot.color_sensor.get_color()!="black":pass
    # robot.wheels.stop()
    # robot.turn_to_direction(20)
    # robot.drive_in_direction(20, -1, -20)
    # robot.turn_to_direction(45)
    # robot.drive_in_direction(45, -17, -20)
    # robot.turn_to_direction(0)


def run_two():
    robot = Robot()
    robot.motion_sensor.reset_yaw_angle()
    robot.set_arm_height(55)
    robot.drive_in_direction(0, 32, 20)
    robot.turn_to_direction(12)
    robot.drive_in_direction(15, 3, 20)
    robot.set_arm_height(93, speed=100)
    robot.turn_to_direction(90)
    robot.set_arm_height(280)
    robot.drive_in_direction(90, 45, 20)
    robot.turn_to_direction(140)
    robot.drive_in_direction(140, -16, -20)
    robot.turn_to_direction(120)
    robot.drive_in_direction(105, -25, -20)
    robot.set_arm_height(0)
    robot.drop_block()
    robot.drive_in_direction(105, 5.5, 20)
    robot.drop_block()
    robot.drive_in_direction(105, 4.5, 20)
    robot.drop_block()
    robot.drive_in_direction(105, 6.5, 20)
    robot.drop_block()
    robot.wheels.start(0, 20)
    while robot.color_sensor.get_color() != "black":
        pass
    robot.wheels.stop()
    robot.turn_to_direction(180)
    robot.set_arm_height(280)
    robot.drive_in_direction(180, -15, -20)
    robot.wheels.start(0, -20)
    while robot.color_sensor.get_color() != "black":
        pass
    robot.wheels.stop()
    robot.drive_in_direction(180, -3, -20)
    robot.turn_to_direction(135)
    robot.drive_in_direction(135, -6.5, -20)
    robot.drop_block()

    robot.drive_in_direction(135, 17, 20)
    robot.turn_to_direction(4)
    robot.set_arm_height(120)
    robot.drive_in_direction(4, 4.5, 30)
    robot.set_arm_height(170)
    robot.drive_in_direction(4, -6.5, -30)
    robot.turn_to_direction(270)
    robot.drive_in_direction(270, -65, -30)
    robot.drive_in_direction(270, 11.3, 30)
    robot.set_arm_height(30)
    robot.turn_to_direction(240)
    robot.drive_in_direction(240, 0.5, 20)

    robot.set_arm_height(0)
    robot.turn_to_direction(200)
    robot.drive_in_direction(200, 5, 20)
    robot.turn_to_direction(235)
    robot.drive_in_direction(235, 7, 20)
    robot.drive_in_direction(235, -0.5, -5)
    # robot.set_arm_height(90)
    # robot.set_arm_height(0)

    robot.set_arm_height(70)
    # robot.motion_sensor.reset_yaw_angle()
    robot.drive_in_direction(235, 0.5, 3)
    robot.set_arm_height(90)
    robot.drive_in_direction(235, 0.5, 3)
    robot.set_arm_height(110)
    robot.drive_in_direction(235, 0.5, 3)
    robot.set_arm_height(130)
    robot.drive_in_direction(235, 6, 3)

    robot.set_arm_height(300)
    robot.drive_in_direction(320, 35, 30)
    robot.turn_to_direction(180)
    while True:
        robot.set_arm_height(random.randrange(300))
        robot.turn_to_direction(random.randrange(160, 200))


def test2():
    robot = Robot()
    robot.motion_sensor.reset_yaw_angle()
    robot.set_arm_height(300)
    robot.turn_to_direction(135)
    robot.drive_in_direction(135, -14, -20)
    robot.drop_block()
    robot.drive_in_direction(135, 17, 30)
    robot.turn_to_direction(4)
    robot.set_arm_height(120)
    robot.drive_in_direction(4, 6.5, 30)
    robot.set_arm_height(170)
    robot.drive_in_direction(4, -6.5, -30)
    robot.turn_to_direction(270)
    robot.drive_in_direction(270, -65, -30)
    robot.drive_in_direction(270, 11.3, 30)
    robot.set_arm_height(30)
    robot.turn_to_direction(240)
    robot.drive_in_direction(240, 0.5, 20)

    robot.set_arm_height(0)
    robot.turn_to_direction(200)
    robot.drive_in_direction(200, 5, 20)
    robot.turn_to_direction(235)
    robot.drive_in_direction(235, 7, 20)
    robot.drive_in_direction(235, -0.5, -5)
    # robot.set_arm_height(90)
    # robot.set_arm_height(0)

    robot.set_arm_height(70)
    # robot.motion_sensor.reset_yaw_angle()
    robot.drive_in_direction(235, 0.5, 3)
    robot.set_arm_height(90)
    robot.drive_in_direction(235, 0.5, 3)
    robot.set_arm_height(110)
    robot.drive_in_direction(235, 0.5, 3)
    robot.set_arm_height(130)
    robot.drive_in_direction(235, 6, 3)
    robot.drive_in_direction(320, 35, 30)


# reset()
# run_one()
run_two()
# test()
# test2()
