import rp2
import network
import ubinascii
import machine
import urequests as requests
import time
from secrets import secrets
import socket
from imu import MPU6050
from machine import Pin, I2C, PWM
import math


max_val = 1700
min_val = 1000

pwm0_us = 0
pwm1_us = 0
pwm2_us = 0
pwm3_us = 0

# Set coun try to avoid possible errors
rp2.country('US')

# I2C connection to Mpu
i2c = I2C(0, sda=Pin(16), scl=Pin(17), freq=400000)
imu = MPU6050(i2c)

# Connection to all the motors
pwm0 =  PWM(Pin(0))
pwm0.freq(50)
pwm1 =  PWM(Pin(2))
pwm1.freq(50)
pwm2 =  PWM(Pin(4))
pwm2.freq(50)
pwm3 =  PWM(Pin(6))
pwm3.freq(50)
pwm0.duty_ns(pwm0_us*1000)
pwm1.duty_ns(pwm1_us*1000)
pwm2.duty_ns(pwm2_us*1000)
pwm3.duty_ns(pwm3_us*1000)
pwm_states = 'Motor0' + str(pwm0_us) + '  Motor1' + str(pwm1_us) + '  Motor2' + str(pwm2_us) + '  Motor3' + str(pwm3_us)

gravity = 9.80665
mass = 1.1

wlan = network.WLAN(network.STA_IF)
wlan.active(True)
# If you need to disable powersaving mode
# wlan.config(pm = 0xa11140)

# See the MAC address in the wireless chip OTP
mac = ubinascii.hexlify(network.WLAN().config('mac'),':').decode()
print('mac = ' + mac)

# Other things to query
# print(wlan.config('channel'))
# print(wlan.config('essid'))
# print(wlan.config('txpower'))

# Load login data from different file for safety reasons
ssid = secrets['ssid']
pw = secrets['pw']

wlan.connect(ssid, pw)

# Wait for connection with 10 second timeout
timeout = 10
while timeout > 0:
    if wlan.status() < 0 or wlan.status() >= 3:
        break
    timeout -= 1
    print('Waiting for connection...')
    time.sleep(1)
    

# Define blinking function for onboard LED to indicate error codes    
def blink_onboard_led(num_blinks):
    led = machine.Pin('LED', machine.Pin.OUT)
    for i in range(num_blinks):
        led.on()
        time.sleep(.2)
        led.off()
        time.sleep(.2)
    
# Handle connection error
# Error meanings
# 0  Link Down
# 1  Link Join
# 2  Link NoIp
# 3  Link Up
# -1 Link Fail
# -2 Link NoNet
# -3 Link BadAuth

wlan_status = wlan.status()
blink_onboard_led(wlan_status)

if wlan_status != 3:
    raise RuntimeError('Wi-Fi connection failed')
else:
    print('Connected')
    status = wlan.ifconfig()
    print('ip = ' + status[0])
    
# Function to load in html page    
def get_html(html_name):
    with open(html_name, 'r') as file:
        html = file.read()
        
    return html

#MPU calibration function
def mpu_cal(calval):
    print("accel range", imu.accel_range, "\t", "gyro range", imu.gyro_range)
    accel_offset = [0,0,0]
    gyro_offset = [0,0,0]
    #calval = 10
    accel_v = [0,0,0]
    accel_x = [0,0,0]
    gyro_ang = [0,0,0]
    gravity = 9.80665

    for i in range (1,calval):
        accel_offset[0] += imu.accel.x/(calval-1)
        accel_offset[1] += imu.accel.y/(calval-1)
        accel_offset[2] += imu.accel.z/(calval-1)
        gyro_offset[0] += imu.gyro.x/(calval-1)
        gyro_offset[1] += imu.gyro.y/(calval-1)
        gyro_offset[2] += imu.gyro.z/(calval-1)   
        print("accel offset", accel_offset, "gyro offset", gyro_offset)

    return accel_offset, gyro_offset


def mpu_outputs(accel_offset, gyro_offset, accel_v,accel_x,gyro_ang, last_time):
    new_time = time.ticks_ms()
    delta = time.ticks_diff(new_time, last_time)
    last_time = new_time
    accel = [imu.accel.x-accel_offset[0],imu.accel.y-accel_offset[1],imu.accel.z-accel_offset[2]]
    gyro = [imu.gyro.x-gyro_offset[0],imu.gyro.y-gyro_offset[1],imu.gyro.z-gyro_offset[2]]
    
    accel_v =[accel_v[i]+ accel[i]*gravity*delta*0.001 for i in range(3)]
    accel_x =[accel_x[i]+ accel_v[i]*delta*0.001 for i in range(3)]
    gyro_ang = [gyro_ang[i]+gyro[i]*delta*0.001 for i in range(3)]

    print("accel", accel, "gyro", gyro)
    print("accel_v", accel_v, "delta", delta)
    print("accel_x", accel_x, "delta", delta)
    print("gyro_ang", gyro_ang, "delta", delta)
    return accel, accel_v, accel_x, gyro, gyro_ang, last_time

def control_pd(accel_offset, gyro_offset):
    
    myData = open("myData.csv", "at")
    myData.write("Time, Tdelta, x_pos, y_pos, z_pos, x_v, y_v, z_v, "+ \
                 "phi, theta, psi, phi_v, theta_v, psi_v, "+ \
                 "w0, w1, w2, w3, pwm0, pwm1, pw2, pwm3\n")

    gravity = 9.80665
    mass = 1.1
    l = 0.254    #length of drone arm
    k = 2.980e-6 #lift constant
    b = 1.140e-7 #drag constant
    II = [0.05,0.05,0.05]
    accel_v = [0.0,0.0,0.0]
    accel_x = [0.0,0.0,0.0]
    gyro_ang = [0.0,0.0,0.0]
    accel_vd = [0.0,0.0,0.0]
    accel_xd = [0.0,0.0,0.0]
    gyro_d = [0.0,0.0,0.0]
    gyro_angd = [0.0,0.0,0.0]
    
    K_accel_D = [2.0,2.0,2.0]
    K_accel_P = [6.0,6.0,6.0]
    #K_accel_I = [0.5,0.5,0.5]
    K_gyro_D = [2.0,2.0,2.0]
    K_gyro_P = [6.0,6.0,6.0]
    #K_gyro_I = [1.5,1.5,1.5]
    
    last_time = time.ticks_ms()
    
    for i in range (1,100):
    
        new_time = time.ticks_ms()
        delta = time.ticks_diff(new_time, last_time)
        last_time = new_time
        
        accel = [imu.accel.x-accel_offset[0],imu.accel.y-accel_offset[1],imu.accel.z-accel_offset[2]]
        gyro = [imu.gyro.x-gyro_offset[0],imu.gyro.y-gyro_offset[1],imu.gyro.z-gyro_offset[2]]
        accel_v =[accel_v[i]+ accel[i]*gravity*delta*0.001 for i in range(3)]
        accel_x =[accel_x[i]+ accel_v[i]*delta*0.001 for i in range(3)]
        gyro_ang = [gyro_ang[i]+gyro[i]*delta*0.001 for i in range(3)]
        
        print("accel", accel, "gyro", gyro)
        print("accel_v", accel_v, "delta", delta)
        print("accel_x", accel_x, "delta", delta)
        print("gyro_ang", gyro_ang, "delta", delta)
            
        gyro_rad = [i*math.pi/180 for i in gyro]
        gyro_ang_rad = [i*math.pi/180 for i in gyro_ang]
        #gyro_ang_rad[2] = 0.0 # no update for psi angle
        
        T_PD = (gravity*mass/math.cos(gyro_ang_rad[0])/math.cos(gyro_ang_rad[1]))
                
        torq_PD = [(K_gyro_D[i]*(gyro_d[i]-gyro_rad[i])+K_gyro_P[i]*(gyro_angd[i]-gyro_ang_rad[i]))*II[i] for i in range(3)]
        
        w1 = math.sqrt(T_PD/4/k-torq_PD[0]/2/k/l+torq_PD[2]/2/b)
        w0 = math.sqrt(T_PD/4/k-torq_PD[1]/2/k/l-torq_PD[2]/2/b)
        w3 = math.sqrt(T_PD/4/k+torq_PD[0]/2/k/l+torq_PD[2]/2/b)
        w2 = math.sqrt(T_PD/4/k+torq_PD[1]/2/k/l-torq_PD[2]/2/b)
        
        pwm0_pd = w0*1.55
        pwm1_pd =w1*1.45
        pwm2_pd =w2*1.15
        pwm3_pd = w3*1.41
        
        if(pwm0_pd<1200):
            pwm0_pd = 1200
        if(pwm1_pd<1200):
            pwm1_pd = 1200
        if(pwm2_pd<900):
            pwm2_pd = 900
        if(pwm3_pd<1200):
            pwm3_pd = 1200
            
        if(pwm0_pd>1600):
            pwm0_pd = 1600
        if(pwm1_pd>1600):
            pwm1_pd = 1600
        if(pwm2_pd>1300):
            pwm2_pd = 1300
        if(pwm3_pd>1600):
            pwm3_pd = 1600
        
        pwm0.duty_ns(int(pwm0_pd*1000))
        pwm1.duty_ns(int(pwm1_pd*1000))
        pwm2.duty_ns(int(pwm2_pd*1000))
        pwm3.duty_ns(int(pwm3_pd*1000))
        
        myData.write("{0}, {1:8.2f}, {2:5.2f}, {3:5.2f}, {4:5.2f}, {5:5.2f}, {6:5.2f}, {7:5.2f}, {8:5.2f}, {9:5.2f}, {10:5.2f}, {11:5.2f}, {12:5.2f}, {13:5.2f}, {14:5.0f}, {15:5.0f}, {16:5.0f}, {17:5.0f}, {18:5.0f}, {19:5.0f}, {20:5.0f}, {21:5.0f}\n"\
                     .format(last_time, delta, accel_x[0], accel_x[1], accel_x[2], \
                             accel_v[0], accel_v[1], accel_v[2], gyro_ang[0], gyro_ang[1],\
                             gyro_ang[2], gyro[0], gyro[1], gyro[2], w0, w1, w2, w3, pwm0_pd,\
                             pwm1_pd, pwm2_pd, pwm3_pd))
        
                     
        #myData.flush()
            
        print(" pwm0-3 : ",delta, " ", pwm0_pd," ", pwm1_pd," ", pwm2_pd," ", pwm3_pd)
        #print("")

            
        #time.sleep(0.005)
    
    myData.close()
    pwm0_pd = 0
    pwm1_pd = 0
    pwm2_pd = 0
    pwm3_pd = 0
    
    pwm0.duty_ns(pwm0_us*00)
    pwm1.duty_ns(pwm1_us*00)
    pwm2.duty_ns(pwm2_us*00)
    pwm3.duty_ns(pwm3_us*00)
    return pwm0_pd,pwm1_pd,pwm2_pd,pwm3_pd


# Motor calibrations
def pwm_cal():
    return 700, 700, 500, 700

    
# STOP
def pwm_stop():
    print('anish is awesome')
    return 0, 0, 0, 0

def pwm_hover_min():
    return 1466, 1378, 1069, 1345

def pwm_hover_max():
    return 0, 0, 0, 0

def pwm_start():
    return 960,960, 695, 960

# HTTP server with socket
addr = socket.getaddrinfo('0.0.0.0', 80)[0][-1]

s = socket.socket()
s.bind(addr)
s.listen(1)

print('Listening on', addr)
led = machine.Pin('LED', machine.Pin.OUT)

# Listen for connections
while True:
    try:
        cl, addr = s.accept()
        print('Client connected from', addr)
        r = cl.recv(1024)
        # print(r)
        
        r = str(r)
        led_on = r.find('?led=on')
        led_off = r.find('?led=off')
        cal_mpu = r.find('?mpu=cal')
        cal_pwm = r.find('?pwm=cal')
        stop_pwm = r.find('?pwm=stop')
        start_pwm = r.find('?pwm=start')
        hover_min_pwm = r.find('?pwm=hover_min')
        hover_max_pwm = r.find('?pwm=hover_max')
        pwm0_up10 = r.find('?pwm0=up10')
        pwm1_up10 = r.find('?pwm1=up10')
        pwm2_up10 = r.find('?pwm2=up10')
        pwm3_up10 = r.find('?pwm3=up10')
        pwm0_up1 = r.find('?pwm0=up1')
        pwm1_up1 = r.find('?pwm1=up1')
        pwm2_up1 = r.find('?pwm2=up1')
        pwm3_up1 = r.find('?pwm3=up1')
        pwm0_down10 = r.find('?pwm0=down10')
        pwm1_down10 = r.find('?pwm1=down10')
        pwm2_down10 = r.find('?pwm2=down10')
        pwm3_down10 = r.find('?pwm3=down10')
        pwm0_down1 = r.find('?pwm0=down1')
        pwm1_down1 = r.find('?pwm1=down1')
        pwm2_down1 = r.find('?pwm2=down1')
        pwm3_down1 = r.find('?pwm3=down1')
        pd_hover = r.find('?pd=hover')
        
        print('led_on = ', led_on)
        print('led_off = ', led_off)
        print('cal_mpu = ', cal_mpu)
        print('pwm_cal = ', cal_pwm)
        print('pwm_stop = ', stop_pwm)
        print('pd_hover = ', pd_hover)
        
            
        if led_on == 10:
            print('LED ON')
            led.value(1)
            
        if led_off == 10:
            print('LED OFF')
            led.value(0)
        
        if cal_mpu == 10:
            print('Calibrate MPU')
            mpu_offsets = mpu_cal(10)
            
        if pd_hover == 10:
            pd_hover = -1
            mpu_offsets = mpu_cal(10)
            pwmval = pwm_cal()
            pwmval = control_pd(mpu_offsets[0], mpu_offsets[1])
            pwm0_us = 0
            pwm1_us = 0
            pwm2_us = 0
            pwm3_us = 0
        
        if cal_pwm == 10:
            print('Calibrate Motors')
            pwmval = pwm_cal()
            pwm0_us = pwmval[0]
            pwm1_us = pwmval[1]
            pwm2_us = pwmval[2]
            pwm3_us = pwmval[3]
            
        if stop_pwm == 10:
            print('Emergency Stop')
            pwmval = pwm_stop()
            pwm0_us = pwmval[0]
            pwm1_us = pwmval[1]
            pwm2_us = pwmval[2]
            pwm3_us = pwmval[3]
        
        if start_pwm == 10:
            pwmval = pwm_start()
            pwm0_us = pwmval[0]
            pwm1_us = pwmval[1]
            pwm2_us = pwmval[2]
            pwm3_us = pwmval[3]
            
        if hover_min_pwm == 10:
            pwmval = pwm_hover_min()
            pwm0_us = pwmval[0]
            pwm1_us = pwmval[1]
            pwm2_us = pwmval[2]
            pwm3_us = pwmval[3]
            
        if hover_max_pwm == 10:
            pwmval = pwm_hover_max()
            pwm0_us = pwmval[0]
            pwm1_us = pwmval[1]
            pwm2_us = pwmval[2]
            pwm3_us = pwmval[3]
            
        if pwm0_up10 == 10:
            pwm0_us += 10
        if pwm0_up1 == 10:
            pwm0_us += 1
        if pwm0_down10 == 10:
            pwm0_us -= 10
        if pwm0_down1 == 10:
            pwm0_us -= 1

        if pwm1_up10 == 10:
            pwm1_us += 10
        if pwm1_up1 == 10:
            pwm1_us += 1
        if pwm1_down10 == 10:
            pwm1_us -= 10
        if pwm1_down1 == 10:
            pwm1_us -= 1          
            
        if pwm2_up10 == 10:
            pwm2_us += 10
        if pwm2_up1 == 10:
            pwm2_us += 1
        if pwm2_down10 == 10:
            pwm2_us -= 10
        if pwm2_down1 == 10:
            pwm2_us -= 1           
            
        if pwm3_up10 == 10:
            pwm3_us += 10
        if pwm3_up1 == 10:
            pwm3_us += 1
        if pwm3_down10 == 10:
            pwm3_us -= 10
        if pwm2_down1 == 10:
            pwm2_us -= 1
            
        if pwm0_us >= max_val:
            pwm0_us = max_val
        if pwm1_us >= max_val:
            pwm1_us = max_val
        if pwm2_us >= max_val:
            pwm2_us = max_val
        if pwm3_us >= max_val:
            pwm3_us = max_val

        pwm0.duty_ns(pwm0_us*1000)
        pwm1.duty_ns(pwm1_us*1000)
        pwm2.duty_ns(pwm2_us*1000)
        pwm3.duty_ns(pwm3_us*1000)
        
        print('pwm0 =', pwm0_us)
        print('pwm1 =', pwm1_us)
        print('pwm2 =', pwm2_us)
        print('pwm3 =', pwm3_us)
        pwm_states = 'Motor0: ' + str(pwm0_us) + '    Motor1: ' + str(pwm1_us) + '    Motor2: ' + str(pwm2_us) + '    Motor3: ' + str(pwm3_us)+ '\n'  
                     #+ 'accel: ' + str(round(accel[0])) + str(round(accel[1])) + str(round(accel[2])) + '\n' \
                     #+ 'accel_v: ' + str(round(accel_v[0])) + '  ' + str(round(accel_v[1])) + '  ' + str(round(accel_v[2])) + '\n' \
                     #+ 'accel_x: ' + str(round(accel_x[0])) + '  ' + str(round(accel_x[1])) + '  ' + str(round(accel_x[2])) + '\n' \
                     #+ 'gyro: ' + str(round(gyro[0])) + '  ' + str(round(gyro[1])) + '  ' +str(round(gyro[2])) + '\n' \
                     #+ 'gyro_ang: ' + str(round(gyro_ang[0])) + '  ' + str(round(gyro_ang[1])) + '  ' + str(round(gyro_ang[2])) + '\n' #\
                     #+ 'w0: ' +  str(round(w0)) + '  ' str(round(w1)) + '  ' + str(round(w2)) + '  ' + str(round(w3))

        
        response = get_html('index.html') % pwm_states
       
        cl.send('HTTP/1.0 200 OK\r\nContent-type: text/html\r\n\r\n')
        cl.send(response)
        cl.close()
        
    except OSError as e:
        pwm0.duty_ns(0*1000)
        pwm1.duty_ns(0*1000)
        pwm2.duty_ns(0*1000)
        pwm3.duty_ns(0*1000)
        cl.close()
        print('Connection closed')
        
        
pwm0.duty_ns(0*1000)
pwm1.duty_ns(0*1000)
pwm2.duty_ns(0*1000)
pwm3.duty_ns(0*1000)


# Make GET request
#request = requests.get('http://www.google.com')
#print(request.content)
#request.close()