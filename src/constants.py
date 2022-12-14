from devices import UINT16_MAX


PinButtonSTOP = 0
PinButtonMODE = 1
PinEAWinch = 2
PinEBWinch = 3
PinForwardLimit = 4
PinReverseLimit = 5
PinTRIGleft = 6
PinECHOleft = 7
PinTRIGright = 8
PinECHOright = 9
PinENWinch = 10
PinIN1winch = 11
PinIN2winch = 12
PinENtraversal = 13
PinIN1traversal = 14
PinIN2traversal = 15
PinServo = 16
PinIN1spin = 18
PinIN2spin = 19
PinENspin = 20
# PinDIR = 17
# PinSTEP = 18
# PinENABLE_INV = 19
PinIN2arm = 20
PinIN1arm = 21
PinENarm = 22
PinInternalLED = 25
PinEBarm = 26
PinEAarm = 27

# duty cycle for servo open and close (float)
SERVO_OPEN_DUTY = 6000
SERVO_CLOSE_DUTY = 2000
SERVO_DURATION_SEC = 2

WINCH_FREQ = 100
WINCH_DUTY_U16 = 65530
WINCH_UP_POS = 5000
WINCH_DOWN_POS = 0
WINCH_DURATION_MAX_MS = 8*1000

TRAVERSAL_FREQ = 100
TRAVERSAL_DUTY_U16 = 65530
TRAVERSAL_FORWARD_DURATION_SEC = 40
TRAVERSAL_BACKWARD_DURATION_SEC = 40

SPIN_FAST_FREQ = 100
SPIN_FAST_DUTY_U16 = UINT16_MAX - 1
SPIN_FAST_DURATION = 0.5

SPIN_HOLD_FREQ = 100
SPIN_HOLD_DUTY_U16 = int(UINT16_MAX * 0.7)

DEBOUNCE_WAIT_TIME_SEC = 0.2