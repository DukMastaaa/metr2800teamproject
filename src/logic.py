from __future__ import annotations

from abc import ABC, abstractmethod

from micropython import schedule
from machine import Pin, soft_reset
from constants import *
from devices import *
import hcsr04
import utime

# This code uses the "State design pattern", described here
# https://refactoring.guru/design-patterns/state/python/example

class System:
    def __init__(self, state: State):
        self.motor_winch = HBridgeMotor(PinENWinch, PinIN1winch, PinIN2winch)
        self.enc_winch = RotaryEncoder(PinEAWinch, PinEBWinch)

        self.motor_traversal = HBridgeMotor(PinENtraversal, PinIN1traversal, PinIN2traversal)

        self.servo = ServoMotor(PinServo, SERVO_OPEN_DUTY, SERVO_CLOSE_DUTY)

        self.led_internal = Pin(PinInternalLED, mode=Pin.OUT)
        
        self.button_stop = Pin(PinButtonSTOP, mode=Pin.IN)
        self.button_mode = Pin(PinButtonMODE, mode=Pin.IN)

        self._state = None
        self.transition_to(state)

        self.terminate = False
        
        self.shutdown_local = self.shutdown
        self.real_shutdown_local = self.real_shutdown

    def transition_to(self, state: State) -> None:
        """Allows changing state at runtime."""
        self._state = state
        self._state.context = self

    # We forward the rest of the calls to the state object.
    def tick(self) -> None:
        self._state.tick()

    def shutdown(self) -> None:
        schedule(self.real_shutdown_local, None)
    
    def real_shutdown(self, _) -> None:
        self.motor_winch.off()
        # i hope this works??
        soft_reset()


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
        self._sys.enc_winch.zero_counts()

        # interrupt stop button for shutdown
        self._sys.button_stop.irq(self._sys.shutdown_local, Pin.IRQ_FALLING)

        # wait until mode button pressed low
        while self._sys.button_mode.high():
            pass
        self._sys.transition_to(CloseGrabberState())

class CloseGrabberState(State):
    def tick(self) -> None:
        self._sys.servo.close()
        utime.sleep(SERVO_DURATION_SEC)
        self._sys.transition_to(WinchUpState())

class WinchUpState(State):
    def tick(self) -> None:
        self._sys.motor_winch.forward(WINCH_FREQ, WINCH_DUTY_U16)
        while self._sys.enc_winch.count < WINCH_UP_POS:
            pass
        self._sys.motor_winch.off()
        self._sys.transition_to(TraverseForwardState())

class TraverseForwardState(State):
    def tick(self) -> None:
        self._sys.motor_traversal.forward(TRAVERSAL_FREQ, TRAVERSAL_DUTY_U16)
        utime.sleep(TRAVERSAL_FORWARD_DURATION_SEC)
        self._sys.motor_traversal.off()
        self._sys.transition_to(OpenGrabberState())

class OpenGrabberState(State):
    def tick(self) -> None:
        self._sys.servo.open()
        utime.sleep(SERVO_DURATION_SEC)
        self._sys.transition_to(TraverseBackwardState())

class TraverseBackwardState(State):
    def tick(self) -> None:
        self._sys.motor_traversal.backward(TRAVERSAL_FREQ, TRAVERSAL_DUTY_U16)
        utime.sleep(TRAVERSAL_BACKWARD_DURATION_SEC)
        self._sys.motor_traversal.off()
        self._sys.transition_to(FinalState())

class FinalState(State):
    def tick(self) -> None:
        while True:
            self._sys.led_internal.on()
            utime.sleep(0.5)
            self._sys.led_internal.off()
            utime.sleep(0.5)


if __name__ == "__main__":
    # example code on how this would be run
    system = System(InitialState())
    while not system.terminate:
        system.tick()
