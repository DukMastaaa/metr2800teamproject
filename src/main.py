import micropython
from machine import Pin
import hcsr04
import utime
from devices import *
from constants import *
from logic import *

micropython.alloc_emergency_exception_buf(100)

def winch_test():
    motor_winch = HBridgeMotor(PinENWinch, PinIN1winch, PinIN2winch)
    enc_winch = RotaryEncoder(PinEAWinch, PinEBWinch)

    while True:
        print("forward")
        motor_winch.forward(50, pc_to_d(0.5))
        while enc_winch.count < 1000:
            if enc_winch.count % 100 == 0 and enc_winch.update == 1:
                print(enc_winch.count)
                enc_winch.update = 0
        motor_winch.off()
        utime.sleep(2)
        print("backward")
        motor_winch.backward(50, pc_to_d(0.5))
        while enc_winch.count > -1000:
            if enc_winch.count % 100 == 0 and enc_winch.update == 1:
                print(enc_winch.count)
                enc_winch.update = 0
        motor_winch.off()
        utime.sleep(2)

def ultra_test():
    ultra_left = hcsr04.HCSR04(PinTRIGleft, PinECHOleft)
    while True:
        print(ultra_left.distance_mm())
        utime.sleep(0.5)

# def step_test():
#     stepper = StepperMotor(PinENABLE_INV, PinDIR, PinSTEP)
#     while True:
#         steps = int(input("number of steps (signed): "))
#         stepper.n_steps(50, abs(steps), steps < 0)

def limit_test():
    limit_forward = Pin(PinForwardLimit, mode=Pin.OUT)
    while True:
        print("open")
        while limit_forward.value() == 0:
            pass
        print("closed")
        while limit_forward.value() == 1:
            pass

def limit_interrupt_test():
    def callback(_):
        nonlocal updated, value
        updated = 1
        value = limit_forward.value()

    limit_forward = Pin(PinForwardLimit, mode=Pin.OUT)
    updated = 0
    value = 0

    limit_forward.irq(callback, Pin.IRQ_RISING | Pin.IRQ_FALLING, hard=True)
    while True:
        if updated:
            print(value)
            updated = 0

def setpos_test():
    motor_winch = HBridgeMotor(PinENWinch, PinIN1winch, PinIN2winch)
    enc_winch = RotaryEncoder(PinEAWinch, PinEBWinch)

    DUTY = 65530/65535
    FREQ = 50

    while True:
        pos = int(input("enter position: "))

        if enc_winch.count < pos:
            motor_winch.forward(FREQ, pc_to_d(DUTY))
            while enc_winch.count < pos:
                if enc_winch.count % 50 == 0 and enc_winch.update == 1:
                    print(enc_winch.count)
                    enc_winch.update = 0
            motor_winch.off()
        else:
            motor_winch.backward(FREQ, pc_to_d(DUTY))
            while enc_winch.count >= pos:
                if enc_winch.count % 50 == 0 and enc_winch.update == 1:
                    print(enc_winch.count)
                    enc_winch.update = 0
            motor_winch.off()

def traversal_test():
    motor_traversal = HBridgeMotor(PinENtraversal, PinIN1traversal, PinIN2traversal)
    freq = 50
    duty = 65530
    motor_traversal.off()
    utime.sleep(1)
    motor_traversal.backward(freq, duty)
    utime.sleep(2)
    motor_traversal.off()

def spin_test():
    motor_spin = HBridgeMotorPWM(PinENspin, PinIN1spin, PinIN2spin)
    print(f"freq={SPIN_FREQ},duty={SPIN_DUTY_U16}")
    while True:
        duration = float(input("signed duration (s): "))
        if duration >= 0:
            motor_spin.forward(SPIN_FREQ, SPIN_DUTY_U16)
        else:
            motor_spin.backward(SPIN_FREQ, SPIN_DUTY_U16)
        utime.sleep(abs(duration))
        motor_spin.brake()

def spin_repeat():
    motor_spin = HBridgeMotorPWM(PinENspin, PinIN1spin, PinIN2spin)
    duration = float(input("duration (s): "))
    while True:
        motor_spin.forward(SPIN_FREQ, SPIN_DUTY_U16)
        utime.sleep(duration)
        motor_spin.backward(SPIN_FREQ, SPIN_DUTY_U16)
        utime.sleep(duration)
        

def main():
    system = System(InitialState())
    while not system.terminate:
        system.tick()
    led_internal = Pin(25, mode=Pin.OUT)
    while True:
        led_internal.on()
        utime.sleep(0.5)
        led_internal.off()
        utime.sleep(0.5)

if __name__ == "__main__":
    main()
