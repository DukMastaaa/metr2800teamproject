import micropython
from machine import Pin
import hcsr04
import utime
from devices import *
from constants import *

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
        print("backward")
        motor_winch.backward(50, pc_to_d(0.5))
        while enc_winch.count > -1000:
            if enc_winch.count % 100 == 0 and enc_winch.update == 1:
                print(enc_winch.count)
                enc_winch.update = 0

def ultra_test():
    ultra_left = hcsr04.HCSR04(PinTRIGleft, PinECHOleft)
    while True:
        print(ultra_left.distance_mm())
        utime.sleep(0.5)

def step_test():
    stepper = StepperMotor(PinENABLE_INV, PinDIR, PinSTEP)
    while True:
        steps = int(input("number of steps (signed): "))
        stepper.n_steps(50, abs(steps), steps < 0)

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

    DUTY = 0.3
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
    duty = UINT16_MAX // 2
    while True:
        motor_traversal.off()
        utime.sleep(1)
        motor_traversal.forward(freq, duty)
        utime.sleep(3)
        motor_traversal.off()
        utime.sleep(1)
        motor_traversal.backward(freq, duty)
        utime.sleep(3)

def manual_step_test():
    ms = ManualStepper(1, 2, 3, 4, 0, 5)
    while True:
        ms.next_step()
        utime.sleep(1/20)


if __name__ == "__main__":
    # traversal_test()
    setpos_test()
