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
    def __init__(self, use_lift=True):
        self.hub = PrimeHub()
        self.wheels = MotorPair("D", "C")
        self.left_wheel = Motor("D")
        self.right_wheel = Motor("C")
        if use_lift:
            self.lift = Motor("E")

        self.left_color_sensor = ColorSensor("B")
        self.right_color_sensor = ColorSensor("A")
        self.motion_sensor = self.hub.motion_sensor

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
            fred = self.left_color_sensor.get_reflected_light() - 50
            fred = fred * 0.7
            fred = round(fred)
            self.wheels.start(steering=fred, speed=speed)
        self.wheels.stop()

    def follow_line_with_two_sensors(self, distance, speed):
        self.reset_distance_travelled()
        while self.distance_travelled() < distance:
            l = self.left_color_sensor.get_reflected_light()
            r = self.right_color_sensor.get_reflected_light()
            diff = l - r
            steer = round(diff * 1.3)
            print("left = {}, right = {}, steer = {}".format(l, r, steer))
            if steer > 1 or steer < -1:
                steer = 30 if steer > 30 else steer
                steer = -30 if steer < -30 else steer
            self.wheels.start(steering=steer, speed=speed)
        else:
            self.wheels.start(steering=0, speed=speed)
        self.wheels.stop()

    def pid_follow_line_two_sensors(self, distance, speed):
        self.reset_distance_travelled()

        CS_L = self.left_color_sensor
        CS_R = self.right_color_sensor
        motor_p = self.wheels

        speed = 30
        d = 100
        i = 0.0025
        p = 0.7
        t = 25
        dif_p = 0
        pid_i = 0
        while self.distance_travelled() < distance:
            Col_L = CS_L.get_reflected_light()
            Col_R = CS_R.get_reflected_light()

            dif = Col_L - Col_R

            pid_i = i * (pid_i + (dif * t))
            pid_p = dif * p
            pid_d = ((dif - dif_p) / t) * d
            steer = round(pid_p + pid_i + pid_d)
            if steer > 100:
                steer = 100
            elif steer < -100:
                steer = -100

            # print(Col_L, " ", Col_R, " ", pid_p, " ", pid_i, " ", pid_d, " ", steer)
            if steer < 40 and steer > -40:
                motor_p.start(steering=steer, speed=speed)
            elif steer < 60 and steer > -60:
                motor_p.start(steering=steer, speed=round(speed / 2.25))
            elif steer < 70 and steer > -70:
                motor_p.start(steering=steer, speed=round(speed / 2.75))
            else:
                motor_p.start(steering=steer, speed=round(speed / 3.25))

            if dif > 0:
                self.hub.light_matrix.show_image("ARROW_W")
            else:
                self.hub.light_matrix.show_image("ARROW_E")
            dif_p = dif
            # wait_for_seconds(0.025)

        motor_p.stop()

    def turn_to_direction(self, degree):
        while abs(self.motion_sensor.get_yaw_angle() % 360 - degree % 360) > 1:
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

    def pid_turn_to_direction(self, degree):
        if degree > 180:
            degree = -360 + degree
        elif degree < -180:
            degree = 360 + degree
        d = 150
        t = 25
        pid_i = 0
        dif_p = 0

        i = 0.075
        turn = 32
        yaw_angle = self.motion_sensor.get_yaw_angle()
        while 1 != turn and 0 != turn and -1 != turn:
            yaw_angle = self.motion_sensor.get_yaw_angle()
            if yaw_angle >= 0:
                if degree < 0:
                    turn = yaw_angle + (degree * -1)
                    if turn < 180:
                        turn = turn * -1
                    else:
                        turn = 360 - turn
                else:
                    turn = degree - yaw_angle
            else:
                if degree > 0:
                    turn = yaw_angle + (degree * -1)
                    if turn > -180:
                        turn = turn * -1
                    else:
                        turn = -360 - turn
                else:
                    turn = degree - yaw_angle

            pid_i = i * (pid_i + (turn * t))
            pid_d = ((turn - dif_p) / t) * d
            # print(turn, pid_i,pid_d)

            turn_ = turn + pid_i + pid_d
            if turn_ > 100:
                turn_ = 100
            elif turn_ < -100:
                turn_ = -100

            self.wheels.start(steering=100, speed=round(turn_ / 3))
            wait_for_seconds(0.025)
            dif_p = turn

        self.wheels.stop()

    def pid_drive_in_direction(self, direction, distance, speed):
        pid_turn_to_direction(degree)
        self.reset_distance_travelled()

        if degree > 180:
            degree = -360 + degree
        elif degree < -180:
            degree = 360 + degree
        d = 150
        t = 25
        pid_i = 0
        dif_p = 0

        i = 0.075
        turn = 32
        yaw_angle = self.motion_sensor.get_yaw_angle()
        while True:
            if distance < 0 and self.distance_travelled() < distance:
                break
            if distance > 0 and self.distance_travelled() > distance:
                break
            yaw_angle = hub.motion_sensor.get_yaw_angle()
            if yaw_angle >= 0:
                if degree < 0:
                    turn = yaw_angle + (degree * -1)
                    if turn < 180:
                        turn = turn * -1
                    else:
                        turn = 360 - turn
                else:
                    turn = degree - yaw_angle
            else:
                if degree > 0:
                    turn = yaw_angle + (degree * -1)
                    if turn > -180:
                        turn = turn * -1
                    else:
                        turn = -360 - turn
                else:
                    turn = degree - yaw_angle

            pid_i = i * (pid_i + (turn * t))
            pid_d = ((turn - dif_p) / t) * d
            # print(turn, pid_i,pid_d)

            turn_ = turn + pid_i + pid_d
            if turn_ > 100:
                turn_ = 100
            elif turn_ < -100:
                turn_ = -100

            self.wheels.start(steering=round(turn_ / 4), speed=speed)
            # wait_for_seconds(0.025)
            dif_p = turn

        motor_p.stop()


def test1():
    robot = Robot(use_lift=False)
    robot.motion_sensor.reset_yaw_angle()
    speed = 40

    robot.turn_to_direction(11)
    robot.drive_in_direction(11, 54, speed)
    robot.drive_in_direction(11, -15, -1 * speed)
    robot.turn_to_direction(-20)
    robot.drive_in_direction(-20, 45, speed)
    robot.turn_to_direction(40)
    robot.drive_in_direction(40, 45, speed)
    robot.turn_to_direction(130)
    robot.drive_in_direction(130, 44, speed)
    # robot.wheels.move(8, steering=-40, speed=speed)
    # robot.wheels.start(steering=-40, speed=speed)
    # while robot.right_color_sensor.get_color() != "black":
    #     pass
    # robot.wheels.move(1.8, steering=-55, speed=speed)
    # robot.drive_in_direction(65, 5, speed)
    # robot.pid_follow_line_two_sensors(33, speed)

    # robot.turn_to_direction(70)
    # robot.drive_in_direction(70, 35, speed)

    robot.turn_to_direction(-128)
    robot.drive_in_direction(-128, -56, -1 * speed)
    robot.turn_to_direction(-140)
    robot.drive_in_direction(-140, -23, -1 * speed)

    robot.left_wheel.run_for_rotations(3, speed=50)
    robot.drive_in_direction(220, 5, speed)
    robot.turn_to_direction(190)
    robot.drive_in_direction(190, 12, speed)
    robot.turn_to_direction(225)
    robot.drive_in_direction(225, 5, speed)
    robot.turn_to_direction(310)
    robot.drive_in_direction(310, 32, speed)
    robot.turn_to_direction(0)
    robot.drive_in_direction(0, 15, speed)
    robot.wheels.move(10, steering=-60, speed=speed)
    return
    robot.turn_to_direction(250)
    robot.drive_in_direction(250, 15, speed)
    robot.pid_follow_line_two_sensors(52, speed)
    robot.turn_to_direction(270)
    robot.drive_in_direction(270, 25, speed)
    robot.turn_to_direction(220)
    robot.drive_in_direction(220, 70, speed)


def test2():
    robot = Robot()
    robot.motion_sensor.reset_yaw_angle()
    speed = 30

    robot.lift.run_for_rotations(-1, speed=100)

    robot.drive_in_direction(0, 50, speed)
    robot.turn_to_direction(-40)
    robot.drive_in_direction(-40, 40, speed)
    robot.turn_to_direction(-85)
    robot.drive_in_direction(-85, -5, -1 * speed)
    robot.lift.run_for_rotations(1, speed=100)
    robot.drive_in_direction(-85, 12, speed)
    # robot.drive_in_direction(-85, -0.5, -1*speed)

    robot.lift.run_for_rotations(-4.1, speed=100)
    robot.lift.run_for_rotations(0.7, speed=100)

    robot.drive_in_direction(-85, -8, -1 * speed)

    robot.lift.run_for_rotations(2, speed=100)
    robot.turn_to_direction(-30)
    robot.drive_in_direction(-30, 3, speed)
    robot.lift.run_for_rotations(-2, speed=100)

    robot.turn_to_direction(65)
    robot.lift.run_for_rotations(3, speed=100)

    robot.wheels.start(steering=0, speed=speed)
    while robot.right_color_sensor.get_color() != "black":
        pass
    robot.turn_to_direction(40)
    robot.drive_in_direction(40, 15, speed)
    robot.wheels.start(steering=0, speed=20)
    while robot.right_color_sensor.get_color() != "black":
        pass
    robot.wheels.stop()
    robot.turn_to_direction(25)
    robot.drive_in_direction(25, 10, speed)
    robot.lift.run_for_rotations(-0.5, speed=30)
    robot.drive_in_direction(25, -12, -1 * speed)

    robot.lift.run_for_rotations(-2, speed=100)
    robot.turn_to_direction(40)
    robot.drive_in_direction(40, 45, speed)
    robot.wheels.start(steering=0, speed=speed)
    while robot.left_color_sensor.get_reflected_light() > 20:
        pass
    robot.wheels.stop()
    robot.drive_in_direction(45, 5, speed)

    robot.turn_to_direction(-40)
    robot.drive_in_direction(-40, 2, speed)
    robot.lift.run_for_rotations(2.5, speed=100)
    robot.drive_in_direction(-40, -8, -1 * speed)

    robot.turn_to_direction(-130)
    robot.drive_in_direction(-130, 127, speed=60)
    robot.drive_in_direction(-115, 55, speed=60)


def test3():
    robot = Robot()
    robot.motion_sensor.reset_yaw_angle()
    speed = 30

    robot.turn_to_direction(-15)
    robot.drive_in_direction(-15, 70, speed=50)
    robot.turn_to_direction(11)
    robot.drive_in_direction(11, 34, speed=50)
    robot.turn_to_direction(-30)
    robot.drive_in_direction(-30, -25, speed=-30)
    robot.turn_to_direction(0)
    robot.wheels.move(15, steering=-20, speed=-40)
    robot.drive_in_direction(15, -70, speed=-30)
    robot.turn_to_direction(45)


def test4():
    robot = Robot()
    robot.motion_sensor.reset_yaw_angle()
    speed = 30

    robot.drive_in_direction(0, 5, speed)

    robot.lift.run_for_rotations(-4.1, speed=100)
    robot.lift.run_for_rotations(0.7, speed=100)

    robot.drive_in_direction(0, -5, -1 * speed)


test1()
