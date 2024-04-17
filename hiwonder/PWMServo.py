#!/usr/bin/python3
# encoding: utf-8
import time
# import pigpio
import threading
import Hobot.GPIO as GPIO
class PWM_Servo(object):

    # def __init__(self, pi, pin, freq = 50, min_width = 500, max_width=2500, deviation = 0, control_speed = False):
    def __init__(self, pin, freq = 50, min_width = 500, max_width=2500, deviation = 0, control_speed = False):
        # self.pi = pi
        self.SPin = pin
        self.Position = 1500
        self.positionSet = self.Position
        self.Freq = freq
        self.Min = min_width
        self.Max = max_width

        self.Deviation = deviation

        self.stepTime = 20
        self.positionInc = 0.0
        self.Time = 0
        self.Time_t = 0
        self.incTimes = 0
        self.speedControl = control_speed
        self.positionSet_t = 0
        self.posChanged = False
        self.servoRunning = False
        if control_speed is True:
            t = threading.Thread(target=PWM_Servo.updatePosition, args=(self,))
            t.setDaemon(True)
            t.start()
            # t.join()  

    def setPosition(self, pos, time = 0):
        print("test")
        if pos < self.Min or pos > self.Max:
            return
        if time == 0:
            self.Position = pos
            self.positionSet = self.Position
            PWM_Servo.set_servo_pulsewidth(self.SPin, self.Position + self.Deviation)
            # self.pi.set_PWM_dutycycle(self.SPin, self.Position + self.Deviation)
        else:
            if time < 20:
                self.Time_t = 20
            elif time > 30000:
                self.Time_t = 30000
            else:
                self.Time_t = time
            self.positionSet_t = pos
            self.posChanged = True
            
    def getPosition(self):
        return self.Position

    def set_servo_pulsewidth(Spin, utime):
        GPIO.setmode(GPIO.BOARD)
        pin = Spin + 20
        GPIO.setup(pin, GPIO.OUT)
        t = 0.001 + (utime - 1500) / 1000000
        GPIO.output(pin, GPIO.HIGH)
        time.sleep(t)
        GPIO.output(pin, GPIO.LOW)
        GPIO.cleanup(pin)

    def updatePosition(self):
        while True:
            # print("test2", self.posChanged)
            if self.posChanged is True:
                # print("test2")
                self.Time = self.Time_t
                self.positionSet = self.positionSet_t
                self.posChanged = False
                self.incTimes = int(self.Time / self.stepTime)
                self.positionInc =  self.Position - self.positionSet
                self.positionInc = int(self.positionInc / self.incTimes )
                self.servoRunning = True

            if self.servoRunning is True:
                # print("test1")
                self.incTimes -= 1
                if  self.incTimes <= 0:
                    self.Position = self.positionSet
                    self.servoRunning = False
                else:
                    self.Position = self.positionSet + int(self.positionInc * self.incTimes)
                try:
                    PWM_Servo.set_servo_pulsewidth(self.SPin, int(self.Position + self.Deviation))
                    # self.pi.set_servo_pulsewidth(self.SPin, int(self.Position + self.Deviation))
                # except:
                #     pass
                except Exception as e:
                    print("An error occurred:", e)
            time.sleep(0.02)
            
        def setDeviation(newD = 0):
            if newD > 300 or newD < -300:
                return
            self.Deviation = newD