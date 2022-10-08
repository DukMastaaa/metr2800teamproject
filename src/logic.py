from machine import Pin, PWM
from constants import *
from devices import *
import hcsr04

from __future__ import annotations
from abc import ABC, abstractmethod

# This code uses the "State design pattern", described here
# https://refactoring.guru/design-patterns/state/python/example

class System:
    def __init__(self, state: State):
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

        self.servo = ServoMotor(PinServo, SERVO_OPEN_DUTY, SERVO_CLOSE_DUTY)

        self.led_internal = Pin(PinInternalLED, mode=Pin.OUT)

        self._state = None
        self.transition_to(state)

        self.terminate = False

    def transition_to(self, state: State) -> None:
        """Allows changing state at runtime."""
        self._state = state
        self._state.context = self

    # We forward the rest of the calls to the state object.
    def tick(self) -> None:
        self._state.tick()


class State(ABC):
    @property
    def sys(self) -> System:
        return self._sys

    @sys.setter
    def context(self, sys: System) -> None:
        self._sys = sys

    @abstractmethod
    def tick(self) -> None:
        pass


class InitialState(State):
    def tick(self) -> None:
        # do required initialisation
        self._sys.servo.open()
        self._sys.enc_arm.zero_counts()
        self._sys.enc_winch.zero_counts()

        # what to do with arm extension?
        # position readings are relative from start

        while self._sys.limit_forward.low():
            pass
        self._sys.transition_to(CloseGrabberState())


class CloseGrabberState(State):
    def tick(self) -> None:
        self._sys.servo.close()
        # unfinished


if __name__ == "__main__":
    # example code on how this would be run
    system = System(InitialState())
    while not system.terminate:
        system.tick()
