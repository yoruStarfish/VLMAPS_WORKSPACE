"""Measured-stall assistance with bounded pulses and angular slew limiting."""

import math

from .rotation_assist import RotationAssist


class VelocityCompensator:
    """Pass linear commands through; assist stalled rotation without latching it."""

    def __init__(self, min_wz=0.8, trigger_wz=0.01, low_vx_threshold=0.08,
                 stopped_wz_threshold=0.05, stall_time=0.4,
                 pulse_duration=0.2, cooldown=1.0, odom_timeout=0.3,
                 command_timeout=0.5, max_angular_accel=4.0):
        if not math.isfinite(max_angular_accel) or max_angular_accel <= 0:
            raise ValueError('max_angular_accel must be positive and finite')
        self.assist = RotationAssist(
            min_wz, trigger_wz, low_vx_threshold, stopped_wz_threshold,
            stall_time, pulse_duration, cooldown, odom_timeout, command_timeout)
        self.max_angular_accel = max_angular_accel
        self.last_time = None
        self.output_wz = 0.0

    def update_odom(self, wz, now):
        self.assist.update_odom(wz, now)

    def reset(self):
        self.assist.reset()
        self.assist.command_time = None
        self.last_time = None
        self.output_wz = 0.0

    def apply(self, vx, wz, now):
        if not all(math.isfinite(v) for v in (vx, wz, now)):
            self.reset()
            return 0.0, 0.0
        previous_time = self.last_time
        if (previous_time is not None and
                (now < previous_time or
                 now - previous_time > self.assist.command_timeout)):
            self.reset()
            previous_time = None
        self.last_time = now

        # A request to stop rotating must cancel both a pulse and its slew tail.
        if wz == 0.0:
            self.assist.reset()
            self.assist.command_time = now
            self.output_wz = 0.0
            return vx, 0.0

        target = self.assist.apply(vx, wz, now)
        fresh_odom = (self.assist.odom_time is not None and
                      0 <= now - self.assist.odom_time <= self.assist.odom_timeout)
        if not fresh_odom:
            # Missing feedback must never prolong compensation from an old pulse.
            self.output_wz = wz
            return vx, wz

        if previous_time is None:
            # No elapsed interval exists on the first sample. It cannot start a
            # pulse, so pass the already-smoothed raw command through.
            self.output_wz = wz
            return vx, wz

        # First decelerate to zero on a reversal; do not jump to an opposite
        # compensating pulse. A new direction must satisfy stall_time again.
        if self.output_wz * wz < 0:
            target = 0.0
        dt = min(now - previous_time, 0.1)
        delta = self.max_angular_accel * dt
        self.output_wz += max(-delta, min(delta, target - self.output_wz))
        return vx, self.output_wz
