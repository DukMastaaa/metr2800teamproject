from machine import Pin

touch = Pin(2, Pin.IN)

while True:
    print("on")
    while touch.value() == 1:
        pass
    print("off")
    while touch.value() == 0:
        pass
