# ~/Practicas/Internship/pez_ws/src/bluerobotics_navigator/__init__.py

# stub navigator API for testing

def init():
    print("[navigator stub] init()")

def set_pwm_enable(flag: bool):
    print(f"[navigator stub] set_pwm_enable({flag})")

def set_pwm_freq_hz(hz: float):
    print(f"[navigator stub] set_pwm_freq_hz({hz})")

def set_pwm_channels_values(channels, values):
    formatted = [f"{v:.2f}" for v in values]
    print(f"[navigator stub] set_pwm_channels_values({channels},{formatted})")

def set_pwm_channel_duty_cycle(channel, duty):
    print(f"[navigator stub] set_pwm_channel_duty_cycle({channel},{duty})")

class PwmChannel:
    Ch16 = "Ch16"
    Ch14 = "Ch14"
    Ch15 = "Ch15"
    Ch11 = "Ch11"
    Ch9  = "Ch9"

class Accel:
    x = 0
    y = 0
    z = 0

def read_accel():
    print("[navigator stub] read_accel()")
    return Accel()
