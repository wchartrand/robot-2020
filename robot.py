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

    def follow_line(distance, speed):
        self.reset_distance_travelled()
        while self.distance_travelled() < distance:
            fred = 50 - self.color_sensor.get_reflected_light()
            fred = fred * 0.7
            fred = round(fred)
            self.motor_pair.start(steering=fred, speed=speed)
        self.motor_pair.stop()

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
            T = self.motion_sensor.get_yaw_angle() - direction
            T = T % 360
            if T < 180:
                steering = 0 - T
            else:
                steering = 360 - T
            self.wheels.start(steering=steering * 2, speed=speed)
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
    # robot.drive_in_direction(0, 100, 10)
    robot.set_arm_height(0)


def run_one():
    robot = Robot()
    robot.motion_sensor.reset_yaw_angle()
    # robot.drive_in_direction( 0, 20, 20)
    robot.set_arm_height(300)
    # robot.drive_in_direction( 0, 68, 20)
    robot.drive_in_direction(0, 22, 3)

    robot.drive_in_direction(0, -5, -5)


# test()
run_one()

