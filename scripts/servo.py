import RPi.GPIO as GPIO
import time

class ServoControl(object):
    """
    Class which controls GPIO pins to actuate servo with marker attachment
    """

    def __init__(self, servo_pin=11):
        self.servo_pin = servo_pin

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.servo_pin,GPIO.OUT)
        self.pwm = GPIO.PWM(self.servo_pin,50)

        # calibration points
        self.p1 = (0,2)
        self.p2 = (180,13)

        # up and down states
        self.states = {0:11,1:12}
        self.m = 1.*(self.p2[1]-self.p1[1])/(self.p2[0]-self.p1[0])
        self.state = 0
        self.pwm.start(self.states[self.state])

    def get_angle(self,angle):
        """
        Calculates duty cycle for a given angle between 0-180
        """
        return self.m * (angle - self.p1[0]) + self.p1[1]

    def move_marker(self,command):
        """
        Tries to set the marker down (1) or up (0)
        """
        try:
            self.pwm.ChangeDutyCycle(self.states[int(command)])
        except (KeyError, ValueError):
            print("invalid state!")

    def exit(self):
        self.pwm.stop()
        GPIO.cleanup()
