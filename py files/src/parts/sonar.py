import RPi.GPIO as GPIO
import time


class Sonar:
    def __init__(self, trigger_pin, echo_pin):
        self._trigger_pin = trigger_pin
        self._echo_pin = echo_pin

        self._MAX_DIST = 20 # cm
        self._SPEED_OF_SOUND = 346 # 331 + 0.6*25 = 346m/s
        self._MAX_ECHO_TIME = self._MAX_DIST*1e-2*2 / self._SPEED_OF_SOUND
        self._MAX_PING_DELAY = 18000 # us
        self._max_time = 0
        
        self.distance = 0

        GPIO.setup(self._trigger_pin, GPIO.OUT)
        GPIO.setup(self._echo_pin, GPIO.IN)

        GPIO.output(self._trigger_pin, False)
    
    def trigger(self):
        ##################################################
        ##                   CAUTION                    ##
        ##################################################
        GPIO.output(self._trigger_pin, False)
        time.sleep(0.000004) # 4us
        GPIO.output(self._trigger_pin, True)
        time.sleep(0.00001) # 10us
        GPIO.output(self._trigger_pin, False)

        self._max_time = time.time() + self._MAX_PING_DELAY*1e-6
        while GPIO.input(self._echo_pin) == 0 and time.time() <= self._max_time:
            pass
        while GPIO.input(self._echo_pin) == 1:
            if time.time() > self._max_time:
                return False
        self._max_time = time.time() + self._MAX_ECHO_TIME
        return True
    
    def check(self, event):
        if self.trigger() == False:
            self.distance = self._MAX_DIST # echo pin is not cleared
        while GPIO.input(self._echo_pin) == 0: # check echo pulse width
            if time.time() > self._max_time:
                self.distance = self._MAX_DIST # beyond maximum distance
        self.distance = (time.time() - (self._max_time - self._MAX_ECHO_TIME) - 0.000005) \
                        * self._SPEED_OF_SOUND / 2 * 1e2 # cm