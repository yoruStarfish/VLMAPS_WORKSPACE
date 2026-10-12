"""Bounded starting assistance using measured angular velocity."""

import math


class RotationAssist:
    def __init__(self, min_wz, trigger_wz, low_vx_threshold,
                 stopped_wz_threshold, stall_time, pulse_duration, cooldown,
                 odom_timeout, command_timeout):
        values = (min_wz, trigger_wz, low_vx_threshold, stopped_wz_threshold,
                  stall_time, pulse_duration, cooldown, odom_timeout,
                  command_timeout)
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError('Rotation assistance parameters must be positive and finite')
        if trigger_wz >= min_wz:
            raise ValueError('trigger_wz must be smaller than min_wz')
        self.min_wz = min_wz
        self.trigger_wz = trigger_wz
        self.low_vx_threshold = low_vx_threshold
        self.stopped_wz_threshold = stopped_wz_threshold
        self.stall_time = stall_time
        self.pulse_duration = pulse_duration
        self.cooldown = cooldown
        self.odom_timeout = odom_timeout
        self.command_timeout = command_timeout
        self.odom_time = None
        self.measured_wz = 0.0
        self.command_time = None
        self.direction = 0
        self.stall_since = None
        self.pulse_until = None
        self.next_pulse = 0.0

    def update_odom(self, wz, now):
        if math.isfinite(wz):
            self.measured_wz = wz
            self.odom_time = now
        else:
            self.odom_time = None

    def reset(self):
        self.direction = 0
        self.stall_since = None
        self.pulse_until = None

    def apply(self, vx, wz, now):
        if (self.command_time is None or
                now - self.command_time > self.command_timeout):
            self.reset()
        self.command_time = now
        eligible = (math.isfinite(vx) and math.isfinite(wz) and
                    abs(vx) < self.low_vx_threshold and
                    self.trigger_wz < abs(wz) < self.min_wz and
                    self.odom_time is not None and
                    now - self.odom_time <= self.odom_timeout)
        if not eligible:
            self.reset()
            return wz
        direction = 1 if wz > 0 else -1
        if direction != self.direction:
            self.reset()
            self.direction = direction
        # Once started, sustain the bounded pulse even if odometry reports
        # movement. Eligibility and direction checks above still cancel it.
        if self.pulse_until is not None:
            if now < self.pulse_until:
                return direction * self.min_wz
            self.next_pulse = now + self.cooldown
            self.pulse_until = None
            self.stall_since = None
            return wz
        if abs(self.measured_wz) >= self.stopped_wz_threshold:
            self.stall_since = None
            return wz
        if self.stall_since is None:
            self.stall_since = now
        if (now - self.stall_since >= self.stall_time and
                now >= self.next_pulse):
            self.pulse_until = now + self.pulse_duration
            return direction * self.min_wz
        return wz
