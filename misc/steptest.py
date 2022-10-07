from time import sleep
from machine import Pin, PWM

e = Pin(13)
d = Pin(14)
s = Pin(15)

#while True:
e.high()
d.low()

pwm = PWM(s)
pwm.freq(50)

while True:
    duty = input("enter duty: ")
    pwm.duty_u16(int(duty))
    #sleep(0.01)
