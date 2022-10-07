import micropython
from machine import Pin, PWM
from constants import *

class HBridgeMotor:
    """Motor driven by H-bridge."""
    def __init__(self, enable_num: int, in1_num: int, in2_num: int):
        self.enable_pwm = PWM(Pin(enable_num, mode=Pin.OUT))
        self.in1 = Pin(in1_num, mode=Pin.OUT)
        self.in1.off()
        self.in2 = Pin(in2_num, mode=Pin.OUT)
        self.in2.off()

    def off(self) -> None:
        self.enable_pwm.duty_u16(0)

    def forward(self, freq: int, duty_u16: int) -> None:
        self.enable_pwm.freq(freq)
        self.enable_pwm.duty_u16(duty_u16)
        self.in1.on()
        self.in2.off()

    def backward(self, freq: int, duty_u16: int) -> None:
        self.enable_pwm.freq(freq)
        self.enable_pwm.duty_u16(duty_u16)
        self.in1.off()
        self.in2.on()


class RotaryEncoder:
    """Incremental rotary encoder, driven by interrupts."""
    def __init__(self, ea_num: int, eb_num: int):
        self.ea = Pin(ea_num, mode=Pin.IN)
        self.eb = Pin(eb_num, mode=Pin.IN)

        # passing self.callback to irq() allocates on the heap.
        # we store the handle as an instance variable to avoid this.
        self.callback_local = self.callback
        # use a hardware interrupt on EA rising edge
        self.ea.irq(self.callback_local, Pin.IRQ_RISING, True)

        self.count = 0
        self.update = 0

        print("enc init finished")

    def callback(self, _):
        # executed when A goes high.
        if self.eb.value() == 0:
            self.count += 1
        else:
            self.count -= 1
        self.update = 1


def pc_to_d(percent):
    if percent > 0.9:
        percent = 0.9
    elif percent < 0:
        percent = 0
    return int(65535 * percent)

micropython.alloc_emergency_exception_buf(100)

motor_winch = HBridgeMotor(PinENWinch, PinIN1winch, PinIN2winch)
enc_winch = RotaryEncoder(PinEAWinch, PinEBWinch)

while True:
    print("forward")
    motor_winch.forward(50, pc_to_d(0.5))
    while enc_winch.count < 1000:
        if enc_winch.count % 100 == 0 and enc_winch.update == 1:
            print(enc_winch.count)
            enc_winch.update = 0
    print("backward")
    motor_winch.backward(50, pc_to_d(0.5))
    while enc_winch.count > -1000:
        if enc_winch.count % 100 == 0 and enc_winch.update == 1:
            print(enc_winch.count)
            enc_winch.update = 0
