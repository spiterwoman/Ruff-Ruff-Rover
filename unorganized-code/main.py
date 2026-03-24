import network
import socket
from time import sleep
from picozero import pico_temp_sensor, pico_led
import machine
import rp2
import sys
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
    print("moving forward")
    
def stop():
    Pwm1.duty_u16(0)
    Pwm2.duty_u16(0)
    print("stop moving")

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
    
#while True:
  #  print("Left:", counter1, "Right:", counter2)
    #time.sleep(0.1)
   # move_forward()
   

ssid = 'blanca'
password = '1949194919'

def connect():
    #Connect to WLAN
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(ssid, password)
    while wlan.isconnected() == False:
        if rp2.bootsel_button() == 1:
            sys.exit()
        print('Waiting for connection...')
        pico_led.on()
        sleep(0.5)
        pico_led.off()
        sleep(0.5)
    ip = wlan.ifconfig()[0]
    print(f'Connected on {ip}')
    pico_led.on()
    return ip

def open_socket(ip):
    # Open a socket
    address = (ip, 80)
    connection = socket.socket()
    connection.bind(address) 
    connection.listen(1)
    return connection

def webpage(temperature, state1, state2):
    #Template HTML
    html = f"""
            <!DOCTYPE html>
            <html>
            <form action="./lighton">
            <input type="submit" value="Light on" />
            </form>
            <form action="./lightoff">
            <input type="submit" value="Light off" />
            </form>
            <form action="./move">
            <input type="submit" value="Move Forward" />
            </form>
            <form action="./stop">
            <input type="submit" value="Stop Moving" />
            </form>
            <form action="./close">
            <input type="submit" value="Stop server" />
            </form>
            <p>LED is {state1}</p>
            <p>Robot is {state2}</p>
            <p>Temperature is {temperature}</p>
            </body>
            </html>
            """
    return str(html)


def serve(connection):
    #Start a web server
    LEDstate = 'OFF'
    MoveState = 'stopped'
    pico_led.off()
    temperature = 0
    while True:
        client = connection.accept()[0]
        request = client.recv(1024)
        request = str(request)
        try:
            request = request.split()[1]
        except IndexError:
            pass
        if request == '/lighton?':
            pico_led.on()
            LEDstate = 'ON'
        elif request =='/lightoff?':
            pico_led.off()
            LEDstate = 'OFF'
        elif request =='/move?':
            move_forward()
            MoveState = 'moving'
        elif request =='/stop?':
            stop()
            MoveState = 'stopped'
        elif request == '/close?':
            sys.exit()
        
        temperature = pico_temp_sensor.temp
        html = webpage(temperature, LEDstate, MoveState)
        client.send(html)
        client.close()


ip = connect()
connection = open_socket(ip)
serve(connection)
