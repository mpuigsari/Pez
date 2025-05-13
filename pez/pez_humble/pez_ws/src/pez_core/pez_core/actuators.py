import math

class HopfCPG:
    """
    A simple Hopf‐oscillator CPG.
    - mu = amplitude^2
    - omega = 2π·frequency
    - alpha = convergence rate
    - dt = integration timestep
    """
    def __init__(self, alpha: float, dt: float):
        self.alpha = alpha
        self.dt = dt
        # start on the positive x‐axis
        self.x = 1.0
        self.y = 0.0

    def step(self, mu: float, omega: float):
        """
        Advance one Euler step of the Hopf equations.
        Returns the new x,y state.
        """
        r2 = self.x*self.x + self.y*self.y
        dx = self.alpha * (mu - r2) * self.x - omega * self.y
        dy = self.alpha * (mu - r2) * self.y + omega * self.x

        self.x += dx * self.dt
        self.y += dy * self.dt
        return self.x, self.y


class TailActuator:
    """
    Maps a CPG 'x' output into a PWM command on channel `channel`.
    - default_pwm: the neutral (center) PWM value
    - offset: a PWMValue instance for turning offset
    - navigator: the bluerobotics_navigator API
    """
    def __init__(self, default_pwm, offset_pwm_value, channel, navigator):
        self.default = default_pwm
        self.offset  = offset_pwm_value
        self.channel = channel
        self.nav     = navigator

    def output(self, cpg_x: float):
        """
        Send PWM = center + cpg_x to the tail channel.
        (We assume cpg_x ∈ [–amplitude…+amplitude], 
         so no extra scaling needed here.)
        """
        center = self.default + self.offset.current
        pwm    = int(center + cpg_x)
        self.nav.set_pwm_channels_values([self.channel], [pwm])
        return pwm


class FinActuator:
    """
    Packs your left & right fin PWM into one call.
    """
    def __init__(self, left_pwm_value, right_pwm_value, left_ch, right_ch, navigator):
        self.left   = left_pwm_value
        self.right  = right_pwm_value
        self.lch    = left_ch
        self.rch    = right_ch
        self.nav    = navigator

    def output(self):
        l = int(self.left.current)
        r = int(self.right.current)
        
        self.nav.set_pwm_channels_values([self.lch, self.rch], [l, r])
        return l, r
    
class PWMValue:
    def __init__(self, default, min_, max_, deflect_=0, blend_=0):
        self.default = default
        self.current = default
        self.min     = min_
        self.max     = max_
        self.deflect = deflect_
        self.blend   = blend_

    def clamp(self, value):
        return min(max(value, self.min), self.max)

    def clamp_deflect(self, v):
        return min(max(v, -self.deflect), self.deflect)

class PwmChannel:
    Ch9 = 9-1
    Ch11 = 11-1
    Ch14 = 14-1
    Ch15 = 15-1
    Ch16 = 16-1
