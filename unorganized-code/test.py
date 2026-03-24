from machine import Pin, PWM
import time #importing time for delay  


# Defining motor pins

#OUT1  and OUT2
Pwm1=PWM(Pin(15))
Dir1=Pin(14,Pin.OUT)

Pwm2=PWM(Pin(16))
Dir2=Pin(17,Pin.OUT)

Pwm1.freq(1000)
Pwm2.freq(1000)

# Forward
def move_forward():
    Dir1.high()
    Dir2.high()
    Pwm1.duty_u16(50000)
    Pwm2.duty_u16(50000)
    
def stop():
    Pwm1.duty_u16(0)
    Pwm2.duty_u16(0)

# Define encoder pins
encoder_a = Pin(0, Pin.IN, Pin.PULL_UP)
encoder_b = Pin(1, Pin.IN, Pin.PULL_UP)
encoder_c = Pin(2, Pin.IN, Pin.PULL_UP)
encoder_d = Pin(3, Pin.IN, Pin.PULL_UP)

# Variables
counter1 = 0
counter2 = 0
last_state = 0

# Interrupt handler
def encoder1_handler(pin):
    global counter1, last_state
    
    # Simple direction detection
    if encoder_a.value() != encoder_b.value():
        counter1 += 1
    else:
        counter1 -= 1
    # print(counter) # Optional: print position

# Encoder 2 interrupt
def encoder2_handler(pin):
    global counter2
    if encoder_c.value() != encoder_d.value():
        counter2 += 1
    else:
        counter2 -= 1

# Set interrupt
encoder_a.irq(trigger=Pin.IRQ_RISING, handler=encoder1_handler)
encoder_c.irq(trigger=Pin.IRQ_RISING, handler=encoder2_handler)
    
while True:
    print("Left:", counter1, "Right:", counter2)
    time.sleep(0.1)
    move_forward()
   