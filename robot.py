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


class SpeedManager:
    def __init__(
        self,
        left_wheel,
        right_wheel,
        distance,
        min_speed=20,
        max_speed=100,
        start_ramp_distance=10,
        stop_ramp_distance=20,
        stop_plateau_distance=10,
    ):
        self.left_wheel = left_wheel
        self.right_wheel = right_wheel
        self.distance = abs(distance) - stop_plateau_distance
        self.min_speed = min_speed
        self.max_speed = max_speed
        self.start_ramp_distance = start_ramp_distance
        self.stop_ramp_distance = stop_ramp_distance
        self.stop_plateau_distance = stop_plateau_distance
        self.reset_distance_travelled()

    def reset_distance_travelled(self):
        self.left_wheel.set_degrees_counted(0)
        self.right_wheel.set_degrees_counted(0)

    def distance_travelled(self):
        Turning = self.left_wheel.get_degrees_counted()
        Cards = self.right_wheel.get_degrees_counted()
        average = (-1 * Turning + Cards) / 2 / 360
        distance = average * 9 * pi
        return distance

    def speed(self):
        distance_travelled = abs(self.distance_travelled())
        if distance_travelled > self.distance:
            return self.min_speed
        distance_from_start = abs(distance_travelled)
        max_speed_from_start = (
            self.max_speed
            if distance_from_start > self.start_ramp_distance
            else self.min_speed
            + round(
                (self.max_speed - self.min_speed)
                * (distance_from_start / self.start_ramp_distance)
            )
        )

        distance_from_end = abs(self.distance - distance_travelled)
        max_speed_from_end = (
            self.max_speed
            if distance_from_end > self.stop_ramp_distance
            else self.min_speed
            + round(
                (self.max_speed - self.min_speed)
                * (distance_from_end / self.stop_ramp_distance)
            )
        )

        speed = min(max_speed_from_start, max_speed_from_end)
        return speed


class Robot:
    def __init__(self, use_lift=True):
        self.hub = PrimeHub()
        self.wheels = MotorPair("D", "C")
        self.wheels.set_motor_rotation(9 * pi)
        self.left_wheel = Motor("D")
        self.right_wheel = Motor("C")
        if use_lift:
            self.lift = Motor("E")
            self.small_lift = Motor("F")

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
            # print("left = {}, right = {}, steer = {}".format(l, r, steer))
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
            wait_for_seconds(0.025)

        motor_p.stop()

    def turn_to_direction(self, degree, stop_when_done=True):
        min_speed = 10
        max_speed = 50
        while abs(self.motion_sensor.get_yaw_angle() % 360 - degree % 360) > 1:
            T = self.motion_sensor.get_yaw_angle() - degree
            T = T % 360
            if T < 180:
                steering = -100
            else:
                steering = 100
            speed = abs(T)
            speed = min(speed, max_speed)
            speed = max(speed, min_speed)
            self.wheels.start(steering=steering, speed=speed)
        if stop_when_done:
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

    def drive_in_direction(
        self,
        direction,
        distance,
        speed=None,
        use_pid=True,
        accelerate=True,
        stop_when_done=True,
    ):
        if use_pid:
            self.pid_drive_in_direction(
                direction,
                distance,
                speed=speed,
                accelerate=accelerate,
                stop_when_done=stop_when_done,
            )
            return

        if accelerate:
            self.drive_in_direction_with_acceleration(
                direction, distance, stop_when_done=stop_when_done
            )
            return

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
        if stop_when_done:
            self.wheels.stop()

    def drive_in_direction_with_acceleration(
        self, direction, distance, stop_when_done=True
    ):
        speed_manager = SpeedManager(self.left_wheel, self.right_wheel, distance)
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

            speed = speed_manager.speed()
            if distance < 0:
                steering = -1 * steering
                speed = -1 * speed

            self.wheels.start(steering=steering * 3, speed=speed)
        if stop_when_done:
            self.wheels.stop()

    def pid_drive_in_direction(
        self, degree, distance, speed, accelerate=True, stop_when_done=True
    ):
        speed_manager = None
        if accelerate:
            speed_manager = SpeedManager(self.left_wheel, self.right_wheel, distance)
        else:
            self.reset_distance_travelled()

        speed = lambda: speed_manager.speed() if accelerate else speed
        distance_travelled = (
            lambda: speed_manager.distance_travelled()
            if accelerate
            else self.distance_travelled()
        )

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
            if abs(distance_travelled()) > abs(distance):
                break
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

            turn_ = turn + pid_i + pid_d
            if turn_ > 100:
                turn_ = 100
            elif turn_ < -100:
                turn_ = -100

            speed = speed_manager.speed()
            if distance < 0:
                speed *= -1
            self.wheels.start(steering=round(turn_ / 4), speed=speed())
            wait_for_seconds(0.025)
            dif_p = turn

        if stop_when_done:
            self.wheels.stop()


def test1():
    robot = Robot(use_lift=False)
    robot.motion_sensor.reset_yaw_angle()
    speed = 30

    # Drop off sled
    robot.turn_to_direction(16)
    robot.drive_in_direction(16, 54, speed)
    robot.drive_in_direction(16, -15, -1 * speed)

    # Drive around
    robot.turn_to_direction(-20)
    robot.drive_in_direction(-20, 45, speed)
    robot.turn_to_direction(40)
    robot.drive_in_direction(40, 48, speed)
    robot.turn_to_direction(130)
    robot.drive_in_direction(130, 45, speed)

    robot.turn_to_direction(-135)
    robot.drive_in_direction(-135, -20, -1 * speed)
    robot.turn_to_direction(-128)
    robot.drive_in_direction(-128, -40, -1 * speed)

    robot.turn_to_direction(-135)
    robot.drive_in_direction(-135, -20, -1 * speed)

    target_angle = -135

    # run left wheel for 3 rotations
    # use right wheel to keep straight
    robot.left_wheel.set_degrees_counted(0)
    robot.left_wheel.start(15)
    while robot.left_wheel.get_degrees_counted() < 3 * 360:
        current_angle = robot.motion_sensor.get_yaw_angle()
        angle_diff = current_angle - target_angle
        print("current_angle={}, angle_diff={}".format(current_angle, angle_diff))
        if angle_diff < -3:
            robot.left_wheel.stop()
        else:
            robot.left_wheel.start(15)
        robot.right_wheel.start(round(angle_diff * 2))
    robot.wheels.stop()

    # go home
    robot.drive_in_direction(-135, 200, speed)
    return

    robot.drive_in_direction(220, 5, speed)
    robot.turn_to_direction(190)
    robot.drive_in_direction(190, 12, speed)
    robot.turn_to_direction(225)
    robot.drive_in_direction(225, 5, speed)
    robot.turn_to_direction(310)
    robot.drive_in_direction(310, 32, speed)
    robot.turn_to_direction(0)
    robot.drive_in_direction(0, 15, speed)
    robot.wheels.move(22.4, steering=-70, speed=speed)
    robot.right_wheel.start(speed=10)
    while robot.right_color_sensor.get_reflected_light() > 20:
        pass
    robot.wheels.stop()
    robot.pid_follow_line_two_sensors(58, speed)
    robot.turn_to_direction(-95)
    robot.drive_in_direction(-95, 25, speed)
    robot.turn_to_direction(-135)
    robot.drive_in_direction(-135, 60, speed)
    robot.turn_to_direction(-120)
    robot.drive_in_direction(-120, 50, speed)


def run1():
    robot = Robot()
    robot.motion_sensor.reset_yaw_angle()
    speed = 30

    robot.lift.run_for_rotations(-1, speed=100)

    robot.drive_in_direction(0, 63, speed)
    robot.turn_to_direction(-40)
    robot.drive_in_direction(-40, 27, speed)
    robot.turn_to_direction(-85)
    robot.drive_in_direction(-85, -5, -1 * speed)
    robot.lift.run_for_rotations(1, speed=100)
    robot.drive_in_direction(-85, 12, speed)

    robot.lift.run_for_rotations(-4.1, speed=100)
    robot.lift.run_for_rotations(0.7, speed=100)

    robot.drive_in_direction(-85, -8, -1 * speed)

    robot.lift.run_for_rotations(2, speed=100)
    robot.turn_to_direction(-35)
    robot.drive_in_direction(-35, 3, speed)
    robot.lift.run_for_rotations(-2, speed=100)

    robot.turn_to_direction(65)
    robot.lift.run_for_rotations(3, speed=100)

    robot.wheels.start(steering=0, speed=speed)
    while robot.right_color_sensor.get_color() != "black":
        pass
    robot.turn_to_direction(40)
    robot.drive_in_direction(40, 12, speed)
    robot.wheels.start(steering=0, speed=20)
    while robot.right_color_sensor.get_color() != "black":
        pass
    robot.wheels.stop()
    robot.turn_to_direction(27)
    robot.drive_in_direction(27, 10, speed)
    robot.lift.run_for_rotations(-0.5, speed=30)
    robot.drive_in_direction(25, -12, -1 * speed)

    robot.lift.run_for_rotations(-2, speed=100)
    robot.turn_to_direction(38)
    robot.drive_in_direction(38, 45, speed)
    robot.wheels.start(steering=0, speed=speed)
    while robot.left_color_sensor.get_reflected_light() > 20:
        pass
    robot.wheels.stop()
    robot.drive_in_direction(38, 5, speed)

    robot.turn_to_direction(-40)
    robot.drive_in_direction(-40, 1, speed)
    robot.lift.run_for_rotations(2.5, speed=100)

    robot.turn_to_direction(-60)
    robot.drive_in_direction(-60, -10, -30)
    robot.turn_to_direction(110)
    robot.lift.run_for_rotations(0.5, speed=100)
    robot.drive_in_direction(110, 11, 30)
    robot.turn_to_direction(155)
    robot.drive_in_direction(155, -10, -30)
    robot.turn_to_direction(223)
    robot.drive_in_direction(223, 215, 30)


def run2():
    robot = Robot()
    robot.motion_sensor.reset_yaw_angle()
    speed = 30

    robot.drive_in_direction(0, 10, speed=50)
    robot.turn_to_direction(-15)
    robot.drive_in_direction(-15, 70, speed=50)
    robot.turn_to_direction(16)
    robot.drive_in_direction(16, 20, speed=50)
    robot.wheels.move(10, speed=60)
    robot.drive_in_direction(16, -2, speed=-50)
    robot.turn_to_direction(-30)
    robot.drive_in_direction(-30, -29, speed=-30)
    robot.turn_to_direction(0)
    robot.wheels.move(20, steering=-20, speed=-40)
    robot.turn_to_direction(10)
    robot.drive_in_direction(10, -60, speed=-30)


def run3():
    robot = Robot()
    robot.motion_sensor.reset_yaw_angle()
    speed = 30

    robot.drive_in_direction(0, 25, speed)
    robot.turn_to_direction(10)
    robot.drive_in_direction(10, 12, speed)
    robot.lift.run_for_rotations(-0.5, speed=100)
    robot.lift.run_for_rotations(0.5, speed=100)
    robot.drive_in_direction(10, -23, -1 * speed)


def run4():
    robot = Robot()
    robot.motion_sensor.reset_yaw_angle()
    speed = 30

    robot.drive_in_direction(0, 110, speed)
    robot.drive_in_direction(0, -10, -1 * speed)
    robot.turn_to_direction(-45)
    robot.drive_in_direction(-45, 10, speed)
    robot.turn_to_direction(87)
    robot.drive_in_direction(87, -36, -1 * speed)
    robot.small_lift.run_for_rotations(0.9, speed=30)
    while True:
        robot.small_lift.run_for_rotations(0.01, speed=1)


run1()
