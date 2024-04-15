import rp2
import network
import ubinascii
from machine import PWM,Pin
import urequests as requests
import time

pwm0 =  PWM(Pin(0))
pwm0.freq(50)
pwm1 =  PWM(Pin(2))
pwm1.freq(50)
pwm2 =  PWM(Pin(4))
pwm2.freq(50)
pwm3 =  PWM(Pin(6))
pwm3.freq(50)
def pwm_cal():
   # pwm0.duty_ns(700*1000)
   # pwm1.duty_ns(700*1000)
    pwm2.duty_ns(500*1000)
    pwm3.duty_ns(700*1000)

def stop():
    print('FINISHED RUNNING')
    pwm0.duty_ns(00*1000)
    pwm1.duty_ns(00*1000)
    pwm2.duty_ns(00*1000)
    pwm3.duty_ns(00*1000)
pwm_cal()
print('WORKING')
