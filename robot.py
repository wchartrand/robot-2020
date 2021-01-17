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

# Simple PID Code starts here.
# The following code for Simple PID is from https://pypi.org/project/simple-pid/ and used under licence.

# MIT License
# Copyright (c) 2018 Martin Lundberg
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
# The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

import time


def _clamp(value, limits):
    lower, upper = limits
    if value is None:
        return None
    elif upper is not None and value > upper:
        return upper
    elif lower is not None and value < lower:
        return lower
    return value


try:
    # get monotonic time to ensure that time deltas are always positive
    _current_time = time.monotonic
except AttributeError:
    # time.monotonic() not available (using python < 3.3), fallback to time.time()
    _current_time = time.time


class PID(object):
    """A simple PID controller."""

    def __init__(
        self,
        Kp=1.0,
        Ki=0.0,
        Kd=0.0,
        setpoint=0,
        sample_time=0.01,
        output_limits=(None, None),
        auto_mode=True,
        proportional_on_measurement=False,
        error_map=None,
    ):
        """
        Initialize a new PID controller.
        :param Kp: The value for the proportional gain Kp
        :param Ki: The value for the integral gain Ki
        :param Kd: The value for the derivative gain Kd
        :param setpoint: The initial setpoint that the PID will try to achieve
        :param sample_time: The time in seconds which the controller should wait before generating
            a new output value. The PID works best when it is constantly called (eg. during a
            loop), but with a sample time set so that the time difference between each update is
            (close to) constant. If set to None, the PID will compute a new output value every time
            it is called.
        :param output_limits: The initial output limits to use, given as an iterable with 2
            elements, for example: (lower, upper). The output will never go below the lower limit
            or above the upper limit. Either of the limits can also be set to None to have no limit
            in that direction. Setting output limits also avoids integral windup, since the
            integral term will never be allowed to grow outside of the limits.
        :param auto_mode: Whether the controller should be enabled (auto mode) or not (manual mode)
        :param proportional_on_measurement: Whether the proportional term should be calculated on
            the input directly rather than on the error (which is the traditional way). Using
            proportional-on-measurement avoids overshoot for some types of systems.
        :param error_map: Function to transform the error value in another constrained value.
        """
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.setpoint = setpoint
        self.sample_time = sample_time

        self._min_output, self._max_output = None, None
        self._auto_mode = auto_mode
        self.proportional_on_measurement = proportional_on_measurement
        self.error_map = error_map

        self._proportional = 0
        self._integral = 0
        self._derivative = 0

        self._last_time = None
        self._last_output = None
        self._last_input = None

        self.output_limits = output_limits
        self.reset()

    def __call__(self, input_, dt=None):
        """
        Update the PID controller.
        Call the PID controller with *input_* and calculate and return a control output if
        sample_time seconds has passed since the last update. If no new output is calculated,
        return the previous output instead (or None if no value has been calculated yet).
        :param dt: If set, uses this value for timestep instead of real time. This can be used in
            simulations when simulation time is different from real time.
        """
        if not self.auto_mode:
            return self._last_output

        now = _current_time()
        if dt is None:
            dt = now - self._last_time if now - self._last_time else 1e-16
        elif dt <= 0:
            raise ValueError("dt has negative value {}, must be positive".format(dt))

        if (
            self.sample_time is not None
            and dt < self.sample_time
            and self._last_output is not None
        ):
            # only update every sample_time seconds
            return self._last_output

        # compute error terms
        error = self.setpoint - input_
        d_input = input_ - (
            self._last_input if self._last_input is not None else input_
        )

        # check if must map the error
        if self.error_map is not None:
            error = self.error_map(error)

        # compute the proportional term
        if not self.proportional_on_measurement:
            # regular proportional-on-error, simply set the proportional term
            self._proportional = self.Kp * error
        else:
            # add the proportional error on measurement to error_sum
            self._proportional -= self.Kp * d_input

        # compute integral and derivative terms
        self._integral += self.Ki * error * dt
        self._integral = _clamp(
            self._integral, self.output_limits
        )  # avoid integral windup

        self._derivative = -self.Kd * d_input / dt

        # compute final output
        output = self._proportional + self._integral + self._derivative
        output = _clamp(output, self.output_limits)

        # keep track of state
        self._last_output = output
        self._last_input = input_
        self._last_time = now

        return output

    def __repr__(self):
        return (
            "{self.__class__.__name__}("
            "Kp={self.Kp!r}, Ki={self.Ki!r}, Kd={self.Kd!r}, "
            "setpoint={self.setpoint!r}, sample_time={self.sample_time!r}, "
            "output_limits={self.output_limits!r}, auto_mode={self.auto_mode!r}, "
            "proportional_on_measurement={self.proportional_on_measurement!r},"
            "error_map={self.error_map!r}"
            ")"
        ).format(self=self)

    @property
    def components(self):
        """
        The P-, I- and D-terms from the last computation as separate components as a tuple. Useful
        for visualizing what the controller is doing or when tuning hard-to-tune systems.
        """
        return self._proportional, self._integral, self._derivative

    @property
    def tunings(self):
        """The tunings used by the controller as a tuple: (Kp, Ki, Kd)."""
        return self.Kp, self.Ki, self.Kd

    @tunings.setter
    def tunings(self, tunings):
        """Set the PID tunings."""
        self.Kp, self.Ki, self.Kd = tunings

    @property
    def auto_mode(self):
        """Whether the controller is currently enabled (in auto mode) or not."""
        return self._auto_mode

    @auto_mode.setter
    def auto_mode(self, enabled):
        """Enable or disable the PID controller."""
        self.set_auto_mode(enabled)

    def set_auto_mode(self, enabled, last_output=None):
        """
        Enable or disable the PID controller, optionally setting the last output value.
        This is useful if some system has been manually controlled and if the PID should take over.
        In that case, disable the PID by setting auto mode to False and later when the PID should
        be turned back on, pass the last output variable (the control variable) and it will be set
        as the starting I-term when the PID is set to auto mode.
        :param enabled: Whether auto mode should be enabled, True or False
        :param last_output: The last output, or the control variable, that the PID should start
            from when going from manual mode to auto mode. Has no effect if the PID is already in
            auto mode.
        """
        if enabled and not self._auto_mode:
            # switching from manual mode to auto, reset
            self.reset()

            self._integral = last_output if last_output is not None else 0
            self._integral = _clamp(self._integral, self.output_limits)

        self._auto_mode = enabled

    @property
    def output_limits(self):
        """
        The current output limits as a 2-tuple: (lower, upper).
        See also the *output_limits* parameter in :meth:`PID.__init__`.
        """
        return self._min_output, self._max_output

    @output_limits.setter
    def output_limits(self, limits):
        """Set the output limits."""
        if limits is None:
            self._min_output, self._max_output = None, None
            return

        min_output, max_output = limits

        if None not in limits and max_output < min_output:
            raise ValueError("lower limit must be less than upper limit")

        self._min_output = min_output
        self._max_output = max_output

        self._integral = _clamp(self._integral, self.output_limits)
        self._last_output = _clamp(self._last_output, self.output_limits)

    def reset(self):
        """
        Reset the PID controller internals.
        This sets each term to 0 as well as clearing the integral, the last output and the last
        input (derivative calculation).
        """
        self._proportional = 0
        self._integral = 0
        self._derivative = 0

        self._integral = _clamp(self._integral, self.output_limits)

        self._last_time = _current_time()
        self._last_output = None
        self._last_input = None


# Simple PID Code ends here.
# Beyond this point, the code is not MIT Licenced.


class Robot:
    def __init__(self):
        self.hub = PrimeHub()
        self.wheels = MotorPair("D", "C")
        self.left_wheel = Motor("D")
        self.right_wheel = Motor("C")
        # self.lift = Motor("E")

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

    def pid_follow_line(self, distance, speed, color_sensor=None):
        if color_sensor is None:
            color_sensor = self.left_color_sensor
        self.reset_distance_travelled()
        pid = PID(0.7, 0, 0, setpoint=0, sample_time=None)
        while self.distance_travelled() < distance:
            fred = color_sensor.get_reflected_light() - 50
            steering = pid(fred)
            adjusted_speed = speed
            if abs(steering) > 30:
                adjusted_speed = round(speed / 2)
            print("steering={}, speed={}".format(steering, adjusted_speed))
            self.wheels.start(steering=round(steering), speed=adjusted_speed)

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

            # if steer < 80 and steer > -80:
            #     self.wheels.start(steering=steer, speed=speed)
            # else:
            #     self.wheels.start(steering=steer, speed=int(speed / 2))
        else:
            self.wheels.start(steering=0, speed=speed)
        self.wheels.stop()


def test():
    robot = Robot()
    robot.motion_sensor.reset_yaw_angle()
    speed = 30

    robot.turn_to_direction(10)
    robot.drive_in_direction(10, 54, speed)
    robot.drive_in_direction(10, -15, -1 * speed)
    robot.turn_to_direction(-20)
    robot.drive_in_direction(-20, 45, speed)
    robot.turn_to_direction(40)
    robot.drive_in_direction(40, 45, speed)
    robot.turn_to_direction(130)
    robot.drive_in_direction(130, 33, speed)
    robot.wheels.move(8, steering=-40, speed=speed)
    robot.wheels.start(steering=-40, speed=speed)
    while robot.right_color_sensor.get_color() != "black":
        pass
    robot.wheels.move(1, steering=-40, speed=speed)
    robot.follow_line_with_two_sensors(40, speed)
    robot.turn_to_direction(310)
    robot.turn_to_direction(225)
    robot.drive_in_direction(225, -32, -1 * speed)
    robot.left_wheel.run_for_rotations(3, speed=50)
    robot.drive_in_direction(220, 5, 30)
    robot.turn_to_direction(190)
    robot.drive_in_direction(190, 10, 30)
    robot.turn_to_direction(225)
    robot.drive_in_direction(225, 5, 30)
    robot.turn_to_direction(320)
    robot.drive_in_direction(320, 40, 30)
    robot.turn_to_direction(0)
    robot.drive_in_direction(0, 15, 30)
    robot.turn_to_direction(250)
    robot.drive_in_direction(250, 15, 30)
    robot.follow_line_with_two_sensors(52, speed)
    robot.turn_to_direction(270)
    robot.drive_in_direction(270, 25, speed)
    robot.turn_to_direction(220)
    robot.drive_in_direction(220, 70, speed)


def test2():
    robot = Robot()
    robot.motion_sensor.reset_yaw_angle()
    speed = 30

    # robot.pid_follow_line(200, speed)
    robot.wheels.start(-20, -15)


test2()
