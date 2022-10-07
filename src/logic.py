from machine import Pin, PWM
from constants import *
from devices import *
import hcsr04

from __future__ import annotations
from abc import ABC, abstractmethod

class System:
    def __init__(self):
        self.motor_winch = HBridgeMotor(PinENWinch, PinIN1winch, PinIN2winch)
        self.enc_winch = RotaryEncoder(PinEAWinch, PinEBWinch)

        self.limit_forward = Pin(PinForwardLimit)
        self.limit_reverse = Pin(PinReverseLimit)

        self.ultra_left = hcsr04.HCSR04(PinTRIGleft, PinECHOleft)
        self.ultra_right = hcsr04.HCSR04(PinTRIGright, PinECHOright)

        self.motor_traversal = HBridgeMotor(PinENtraversal, PinIN1traversal, PinIN2traversal)

        self.stepper = StepperMotor(PinENABLE, PinDIR, PinSTEP)

        self.motor_arm = HBridgeMotor(PinENarm, PinIN1arm, PinIN2arm)
        self.enc_arm = RotaryEncoder(PinEAarm, PinEBarm)

        self.servo_pwm = PWM(Pin(PinServo, mode=Pin.OUT))

        self.led_internal = Pin(PinInternalLED, mode=Pin.OUT)

        self._state = None

    def transition_to(self, state: State) -> None:
        """Allows changing state at runtime."""
        self._state = state
        self._state.context = self
    
    # We forward the rest of the calls to the state object.



# https://refactoring.guru/design-patterns/state/python/example
