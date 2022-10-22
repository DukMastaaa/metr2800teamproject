from machine import Pin, PWM, Timer

UINT16_MAX = 65535

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

    def zero_counts(self) -> None:
        self.count = 0


class StepperMotor:
    """
    Stepper motor driven through ENABLE_INV, STEP and DIR pins.
    """
    DUTY = UINT16_MAX // 2

    def __init__(self, enable_num: int, dir_num: int, step_num: int):
        self.step_num = step_num
        self.enable_inv = Pin(enable_num, mode=Pin.OUT)
        self.dir = Pin(dir_num, mode=Pin.OUT)
        # hopefully both of these can run at once
        self.step = Pin(step_num, mode=Pin.OUT)
        self.step_pwm = PWM(Pin(step_num, mode=Pin.OUT))
        self.pwm_active = True

        self.tick_timer = Timer(-1)
        self.tick_value = 0
        self.stop_timer = Timer(-1)

    def activate_pwm(self) -> None:
        if not self.pwm_active:
            self.step_pwm = PWM(Pin(self.step_num))
            self.pwm_active = True

    def activate_manual(self) -> None:
        if self.pwm_active:
            self.step = Pin(self.step_num, mode=Pin.OUT)
            self.pwm_active = False

    def off(self) -> None:
        self.enable_inv.on()
        self.step.off()
        self.step_pwm.duty_u16(0)

    def brake(self) -> None:
        self.enable_inv.off()
        self.step_pwm.deinit()

    def forward(self, freq: int) -> None:
        self.enable_inv.off()
        self.dir.off()
        self.activate_pwm()
        self.step_pwm.freq(freq)
        self.step_pwm.duty_u16(self.DUTY)

    def backward(self, freq: int) -> None:
        self.enable_inv.off()
        self.dir.on()
        self.activate_pwm()
        self.step_pwm.freq(freq)
        self.step_pwm.duty_u16(self.DUTY)

    def tick_callback(self, _) -> None:
        if self.tick_value == 0:
            self.tick_value = 1
            self.step.on()
        else:
            self.tick_value = 0
            self.step.off()

    def stop_callback(self, _) -> None:
        self.tick_timer.deinit()

    def n_steps(self, freq: int, num: int, reverse: bool = False) -> None:
        """Turns the given number of steps then stops."""
        self.enable_inv.on()
        self.activate_manual()
        self.tick_value = 0
        self.dir.value(int(reverse))

        self.tick_timer.init(
            freq=freq,
            mode=Timer.PERIODIC,
            callback=self.tick_callback
        )
        # each on-off pulse is 1/freq*1000*2 ms
        # stop tick_timer after num of these pulses
        self.stop_timer.init(
            period=int(1/freq*1000*2*num),
            mode=Timer.ONE_SHOT,
            callback=self.stop_callback
        )


class ServoMotor:
    def __init__(self, signal_num: int, open_duty: float, close_duty: float):
        self.signal_pwm = PWM(Pin(signal_num, mode=Pin.OUT))
        self.open_duty_u16 = int(UINT16_MAX * open_duty)
        self.close_duty_u16 = int(UINT16_MAX * close_duty)

    def open(self):
        self.signal_pwm.duty_u16(self.open_duty_u16)

    def close(self):
        self.signal_pwm.duty_u16(self.close_duty_u16)


def pc_to_d(percent):
    if percent > 0.9:
        percent = 0.9
    elif percent < 0:
        percent = 0
    return int(65535 * percent)
