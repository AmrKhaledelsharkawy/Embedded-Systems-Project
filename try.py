from machine import Pin, SPI ,UART

led = Pin(0, Pin.OUT)
led2 = Pin(1, Pin.OUT)

led.value(1)
led2.value(1)