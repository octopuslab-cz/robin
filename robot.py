# micropython imports
from machine import Pin, Timer, PWM, SPI
from time import sleep_ms
from random import randrange

# lib imports
from hcsr04 import HCSR04
from colors_rgb import *

# octopus imports
from utils.pinout import set_pinout
from utils.octopus import oled_init
from components.rgb import Rgb
from components.analog import Analog
from components.servo import Servo  # todo: PWM double setup error
from utils.io_config import get_from_file 

battery = Analog(36) # TODO from pinout
pinout = set_pinout()

io_conf = get_from_file()

ws = Rgb(pinout.WS_LED_PIN,io_conf.get('ws'))
ir = Pin(pinout.DEV1_PIN, Pin.IN)

print('oled init start')
OLED_I2C_ADDRESS = 60  # defaul is 0x3c = 60, alternative 35
oled = None
while not oled:
    try:
        oled = oled_init(addr=OLED_I2C_ADDRESS)
        sleep_ms(500)
    except OSError as e:
        print(e)
        print('oled init retry')
print('oled init done')

MAX_SPEED = 650
CRUISING_SPEED = 550
APPROACH_SPEED = 450
TURN_SPEED = 450
TURN_90_DEG_MS = 600  # ms
TURN_180_DEG_MS = 900  # ms
COLLISION_THRESHOLD = 25  # cm
APPROACH_THRESHOLD = 60 #cm
MAX_SPEED_THRESHOLD = 180 #cm
ENABLE_COMPENSATION = True
LR_COMPENSATION = +10  # %, -10 slows L motor comared to R
ULTRASONIC_SAMPLING = 60  # ms, how often detect obstacle

moto_L1 = Pin(pinout.MOTOR_1A, Pin.OUT)
moto_L2 = Pin(pinout.MOTOR_2A, Pin.OUT)
moto_L  = PWM(Pin(pinout.MOTOR_12EN, Pin.OUT), freq=500, duty = 0)


moto_R3 = Pin(pinout.MOTOR_3A, Pin.OUT)
moto_R4 = Pin(pinout.MOTOR_4A, Pin.OUT)
moto_R  = PWM(Pin(pinout.MOTOR_34EN, Pin.OUT), freq=500, duty = 0)

echo = HCSR04(trigger_pin=pinout.PWM2_PIN, echo_pin=pinout.PWM1_PIN)

servo = Servo(pinout.PWM3_PIN)
servo.set_degree(50)

def read_distance(echo):
    d = echo.distance_cm()
    if d < 0:
        sleep_ms(50)
        d = echo.distance_cm()
    print('distance', d)
    return d

def compensate_speed_left(speed):
    return speed + int(speed/100 * LR_COMPENSATION/2) * ENABLE_COMPENSATION

def compensate_speed_right(speed):
    return speed - int(speed/100 * LR_COMPENSATION/2) * ENABLE_COMPENSATION

def forward(speed):
    stop()
    moto_L1.value(0)
    moto_L2.value(1)

    moto_R3.value(0)
    moto_R4.value(1)

    moto_L.duty(compensate_speed_left(speed))
    moto_R.duty(compensate_speed_right(speed))

def backward(speed):
    stop()
    moto_L1.value(1)
    moto_L2.value(0)

    moto_R3.value(1)
    moto_R4.value(0)

    moto_L.duty(compensate_speed_left(speed))
    moto_R.duty(compensate_speed_right(speed))

def stop():
    moto_L.duty(0)
    moto_R.duty(0)


def turn_left(ms):
    stop()
    moto_L1.value(1)
    moto_L2.value(0)

    moto_R3.value(0)
    moto_R4.value(1)

    moto_L.duty(compensate_speed_left(TURN_SPEED))
    moto_R.duty(compensate_speed_right(TURN_SPEED))
    sleep_ms(ms)
    stop()


def turn_right(ms):
    stop()
    moto_L1.value(0)
    moto_L2.value(1)

    moto_R3.value(1)
    moto_R4.value(0)

    moto_L.duty(compensate_speed_left(TURN_SPEED))
    moto_R.duty(compensate_speed_right(TURN_SPEED))
    sleep_ms(ms)
    stop()

def random_turn(ms=randrange(TURN_90_DEG_MS, TURN_180_DEG_MS)):
    if randrange(2):
        turn_left(ms)
    else:
        turn_right(ms)

def set_status(text, color=BLACK):
    ws.color(color)
    print(text)
    oled.fill_rect(0, 21, 128, 10, 0)
    oled.text("{0:^15}".format(text), 3, 21)
    oled.show()

def oled_default():
    oled.clear()
    oled.contrast(10)
    oled.text("{0:^15}".format("Robin 2020"), 3, 1)
    oled.hline(0, 10, 128, 1)
    oled.hline(0, 53, 128, 1)
    oled.show()

def oled_distance(value):
    oled.fill_rect(0, 41, 128, 10, 0)
    oled.text("dist: {0:5.2f} cm".format(value), 3, 41)
    oled.show()

def oled_battery():
    oled.fill_rect(0, 55, 128, 10, 0)
    oled.text("[{0:.1f}]".format(battery.get_adc_aver()), 3, 55)
    oled.show()

def start():
    oled_default()

    cnt = 0
    oled_battery()

    while True:
        try:
            cnt += 1
            if cnt * ULTRASONIC_SAMPLING > 5000:
                cnt = 0
                oled_battery()
            distance = read_distance(echo)
            oled_distance(distance)
            if ir.value() == 0:
                # ir collision detected
                distance = 0.1
            if distance < 0 or distance > MAX_SPEED_THRESHOLD:
                set_status("MAX SPEED", WHITE)
                forward(MAX_SPEED)
            elif distance > APPROACH_THRESHOLD:
                set_status("CRUISE", GREEN)
                forward(CRUISING_SPEED)
            elif distance > COLLISION_THRESHOLD:
                set_status("APPROACH", YELLOW)
                forward(APPROACH_SPEED)
            else:  # distance < COLLISION_THRESHOLD
                set_status("BACK OFF", RED)
                backward(APPROACH_SPEED)
                sleep_ms(500)
                stop()

                # look LEFT
                set_status("look LEFT")
                servo.set_degree(95)
                sleep_ms(500)
                left_distance = read_distance(echo)

                # look RIGHT
                set_status("look RIGHT")
                servo.set_degree(5)
                sleep_ms(500)
                right_distance = read_distance(echo)

                # look STRAIGHT
                set_status("look STRAIGHT")
                servo.set_degree(50)
                sleep_ms(500)

                if left_distance < COLLISION_THRESHOLD:
                    if right_distance < COLLISION_THRESHOLD:
                        # turn around - both directions are occupied
                        set_status("TURN AROUND", BLUE)
                        random_turn(TURN_180_DEG_MS)
                    else:
                        # turn right
                        set_status("TURN RIGHT", BLUE)
                        turn_right(TURN_90_DEG_MS)
                else:
                    if right_distance > COLLISION_THRESHOLD:
                        # turn random - both directions are free
                        set_status("TURN RANDOM", BLUE)
                        random_turn()
                    else:
                        # turn left
                        set_status("TURN LEFT", BLUE)
                        turn_left(TURN_90_DEG_MS)
            sleep_ms(ULTRASONIC_SAMPLING)
        except Exception as e:
            # handle all exceptions
            print(e)
