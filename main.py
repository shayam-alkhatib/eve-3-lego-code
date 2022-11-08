#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Button, Stop, Color, Direction
from pybricks.tools import wait
from pybricks.robotics import DriveBase

# Initialize the EV3 Brick
ev3 = EV3Brick()


# initalizaties
motor1 = Motor(Port.A)
motor2 = Motor(Port.B)
robot = DriveBase(motor1, motor2, wheel_diameter=55.5, axle_track=104)

obstacle_sensor = UltrasonicSensor(Port.S4)
line_sensor = ColorSensor(Port.S2)
touch = TouchSensor(Port.S3)
# global touchbutton
touchButton = "Off"

BLUE = 99
WHITE = 1 
threshold = (BLUE + WHITE) / 2
 
DRIVE_SPEED = 50
PROPORTIONAL_GAIN = 1.2
turn_direction = 1
initial_turn = 30
timer = 0



# function om te checken of de button losgelaten word
def touchcheck():
    global touchButton
    if touch.pressed() is False:
        touchButton = "off"

    elif touch.pressed() is True:
        touchButton = "on"

# speaker/beep
ev3.speaker.set_volume(100)
ev3.speaker.set_speech_options('nl','f4', 100, 75)

#Spreken
ev3.speaker.say("Hallo ik ben Tyr, ik ga je vandaag door het gebouw heen begeleiden.")
ev3.speaker.say("Zodra je één piepje hoort begin ik met rijden en kun je mij volgen.")
ev3.speaker.say("Zodra je twee piepjes hoort stop ik met rijden en moet je stoppen met lopen.")
ev3.speaker.say("Als je de knop indrukt en blijft indrukken begin ik met rijden. Zodra je de knop loslaat stop ik met rijden. Voor de veiligheid gaat er een alarm af als je na dertig seconden de knop nog niet hebt ingedrukt en ingedrukt houdt.")
ev3.speaker.say("Als je klaar bent om te vertrekken kun je de knop indrukken en blijven indrukken nadat je twee piepjes hebt gehoord.")
locatie = 0

if touch.pressed() == True:
    touchButton = "on"


# Loop voor de touch sensor
while locatie == 0:
    touchcheck()
    # print (touchButton)
    if touchButton == "off":
        beep = 0
        if timer == 0:
            ev3.speaker.beep()
            wait(100)
            ev3.speaker.beep()
        robot.stop()
        wait(1000)
        timer += 1
        print(timer)
        if timer >= 30:
            ev3.speaker.beep(800,100)
            wait(5)
            ev3.speaker.beep(800,100)

    if touchButton == "on":
        if beep == 0:
            ev3.speaker.beep()
        timer = 0
        beep = 1
        deviation = line_sensor.reflection() - threshold
        while deviation > 25:
            touchcheck()
            while obstacle_sensor.distance() < 60:
                # print (obstacle_sensor.distance())
                robot.stop()
            if touch.pressed() is False:
                break
            robot.turn(initial_turn * turn_direction)
            deviation = line_sensor.reflection() - threshold
            turn_direction *= -1
            initial_turn += 15
            if initial_turn >= 120:
                initial_turn = 30

        turn_rate = PROPORTIONAL_GAIN * deviation
        robot.drive(DRIVE_SPEED, turn_rate)