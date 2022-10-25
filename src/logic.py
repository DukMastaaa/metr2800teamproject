import os
if os.uname().sysname != "rp2":
    from __future__ import annotations

#from abc import ABC, abstractmethod

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

        self.motor_spin = HBridgeMotorPWM(PinENspin, PinIN1spin, PinIN2spin)

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

    def real_shutdown(self, test) -> None:
        print("SHUTDOWN")
        self.motor_winch.off()
        # i hope this works??
        soft_reset()

    def shutdown(self) -> None:
        schedule(self.real_shutdown_local, 123)


class State():
    @property
    def sys(self) -> System:
        return self._sys

    @sys.setter
    def context(self, sys: System) -> None:
        self._sys = sys

    #@abstractmethod
    def tick(self) -> None:
        pass


class InitialState(State):
    def tick(self) -> None:
        print("InitialState")
        # do required initialisation
        self._sys.servo.open()
        self._sys.enc_winch.zero_counts()

        # change state
        self._sys.transition_to(SelectModeState())

class SelectModeState(State):
    def tick(self) -> None:
        print("SelectModeState")
        # wait until either button is pressed low
        while self._sys.button_mode.value() == 1 and self._sys.button_stop.value() == 1:
            pass
        # wait a tiny bit to hopefully ignore debouncing
        utime.sleep(DEBOUNCE_WAIT_TIME_SEC)

        if self._sys.button_mode.value() == 0 and self._sys.button_stop.value() == 0:
            # wait until both depressed
            while self._sys.button_mode.value() == 0 and self._sys.button_stop.value() == 0:
                pass
            # wait a bit longer for debouncing
            utime.sleep(DEBOUNCE_WAIT_TIME_SEC)
            # go to ready state
            self._sys.transition_to(ReadyState())
        else:
            if self._sys.button_mode.value() == 0:
                # only mode (blue) button pressed
                self._sys.transition_to(CalibrateTraversalState())
            else:
                # only stop (red) button pressed
                self._sys.transition_to(CalibrateWinchState())
            # wait a bit longer for debouncing
            utime.sleep(DEBOUNCE_WAIT_TIME_SEC)

def calibration_helper(sys: System, device, freq, duty_u16,
                       next_state: State):
    """
    Does forward-backward calibration for the given device.
    """
    # wait until either button is pressed low
    while sys.button_mode.value() == 1 and sys.button_stop.value() == 1:
        pass
    # wait a tiny bit to hopefully ignore debouncing
    utime.sleep(DEBOUNCE_WAIT_TIME_SEC)

    if sys.button_mode.value() == 0 and sys.button_stop.value() == 0:
        # if both buttons are pressed, move to ready state
        # wait until both depressed
        while sys.button_mode.value() == 0 and sys.button_stop.value() == 0:
            pass
        # wait a bit longer for debouncing
        utime.sleep(DEBOUNCE_WAIT_TIME_SEC)
        # go to next state
        sys.transition_to(next_state)
    else:
        # only one of the buttons are pressed
        if sys.button_mode.value() == 0:
            # mode (blue) button pressed, traverse forward
            device.forward(freq, duty_u16)
            # keep moving while button pressed
            while sys.button_mode.value() == 0:
                pass
        else:
            # stop (red) button pressed, traverse backward
            device.backward(freq, duty_u16)
            # keep moving while button pressed
            while sys.button_stop.value() == 0:
                pass
        # stop moving
        device.off()
        # wait a bit longer for debouncing
        utime.sleep(DEBOUNCE_WAIT_TIME_SEC)
        # don't change state

class CalibrateTraversalState(State):
    def tick(self) -> None:
        print("CalibrateTraversalState")
        calibration_helper(
            self._sys, self._sys.motor_traversal, TRAVERSAL_FREQ, TRAVERSAL_DUTY_U16,
            SelectModeState()
        )

class CalibrateWinchState(State):
    def tick(self) -> None:
        print("CalibrateWinchState")
        calibration_helper(
            self._sys, self._sys.motor_winch, WINCH_FREQ, WINCH_DUTY_U16,
            SelectModeState()
        )

class ReadyState(State):
    def tick(self) -> None:
        print("ReadyState")
        self._sys.led_internal.on()

        # interrupt stop button for shutdown
        self._sys.button_stop.irq(self._sys.shutdown_local, Pin.IRQ_FALLING)
        # wait for blue (mode) button to be pressed to start
        while self._sys.button_mode.value() == 1:
            pass
        # start!
        self._sys.transition_to(CloseGrabberState())

class CloseGrabberState(State):
    def tick(self) -> None:
        print("CloseGrabberState")
        self._sys.led_internal.off()
        self._sys.servo.close()
        utime.sleep(SERVO_DURATION_SEC)
        self._sys.transition_to(TurnArmState())

class TurnArmState(State):
    def tick(self) -> None:
        print("TurnArmState")
        self._sys.motor_spin.forward(SPIN_FREQ, SPIN_DUTY_U16)
        utime.sleep(SPIN_DURATION)
        self._sys.motor_spin.off()
        self._sys.transition_to(WinchUpState())

class WinchUpState(State):
    def tick(self) -> None:
        print("WinchUpState")
        self._sys.motor_winch.forward(WINCH_FREQ, WINCH_DUTY_U16)
        utime.sleep(WINCH_DURATION_MAX_MS)
        start_time = utime.ticks_ms()
        while self._sys.enc_winch.count < WINCH_UP_POS:
            if utime.ticks_diff(utime.ticks_ms(), start_time) > WINCH_DURATION_MAX_MS:
                print("WinchUp timeout")
                break
            utime.sleep(0.1)  # ticks_diff seems intensive
            if self._sys.enc_winch.count % 100 == 0:
                print(self._sys.enc_winch.count)
        self._sys.motor_winch.off()
        self._sys.transition_to(TraverseForwardState())

class TraverseForwardState(State):
    def tick(self) -> None:
        print("TraverseForwardState")
        self._sys.motor_traversal.forward(TRAVERSAL_FREQ, TRAVERSAL_DUTY_U16)
        utime.sleep(TRAVERSAL_FORWARD_DURATION_SEC)
        self._sys.motor_traversal.off()
        self._sys.transition_to(OpenGrabberState())

class OpenGrabberState(State):
    def tick(self) -> None:
        print("OpenGrabberState")
        self._sys.servo.open()
        utime.sleep(SERVO_DURATION_SEC)
        self._sys.transition_to(TraverseBackwardState())

class TraverseBackwardState(State):
    def tick(self) -> None:
        print("TraverseBackwardState")
        self._sys.motor_traversal.backward(TRAVERSAL_FREQ, TRAVERSAL_DUTY_U16)
        utime.sleep(TRAVERSAL_BACKWARD_DURATION_SEC)
        self._sys.motor_traversal.off()
        self._sys.transition_to(FinalState())

class FinalState(State):
    def tick(self) -> None:
        print("FinalState")
        self._sys.terminate = True
        """
        while True:
            self._sys.led_internal.on()
            utime.sleep(0.5)
            self._sys.led_internal.off()
            utime.sleep(0.5)
        """
