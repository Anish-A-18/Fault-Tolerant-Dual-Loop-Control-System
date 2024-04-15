from mpu6050 import mpu6050
from rpi_hardware_pwm import HardwarePWM
import time
import numpy as np
import math
import csv

f = open('SingleArmData.csv', 'w', newline= '')
write = csv.writer(f)
write.writerow(['time', 'time_change', 'phi velocity', 'phi angle', 'thrust', 'torque', 'w1', 'w3', 'PWM w1', 'PWM w3'])


# MPU_6050 variables
mpu = mpu6050(0x68)
mpu.set_accel_range(0x08)
mpu.set_gyro_range(0x08)
accel_range = mpu.read_accel_range()
gyro_range = mpu.read_gyro_range()

print("Accelerometer Range :" + str(accel_range))
print("Gyroscope Range :" + str(gyro_range))

mpu_accel_array = []
mpu_gyro_array = []

cal_size = 100;
accel_offsets = [0.0, 0.0, 0.0]
gyro_offsets = [0.0, 0.0, 0.0]

# PWM variables
pwm0 = HardwarePWM(pwm_channel=0, hz=50)
pwm1 = HardwarePWM(pwm_channel=1, hz=50)

i = 200
j = 1

pwm0.start(0)
pwm1.start(0)

speed0 = 1700
speed1 = 2000
min_val = 1700
max_val = 2000

max_time_freq = 20000
min_duty = min_val / max_time_freq * 100
max_duty = max_val / max_time_freq * 100

sep_val = 1
com_val = 10
pwm_loop = 1

# PID variables
g = 9.80665  # gravity units: m/s^2
m = 0.615  # mass units: kg
l = 0.254  # distance between motors and center of mass units: m
k = 2.98e-6  # lift constant
b = 1.14e-7  # drag constant
Ixx = 9.256e-3  # inertia for x-dir units: kg*m^2
Iyy = 9.256e-3  # inertia for y-dir units: kg*m^2
Izz = 8.801e-3  # inertia for z-dir units: kg*m^2

Ax = 0.25  # drag force coeffienct for velocity untis: kg/s
Ay = 0.25  # drag force coeffienct for velocity untis: kg/s
Az = 0.25  # drag force coeffienct for velocity untis: kg/s

KphiD = 1.5
KphiP = 6
phi_d = 0  # desired value for angle phi
phi_v_d = 0  # desired value for angular velocity of phi
T_PD = 0  # thrust value for controller
torq_phi_PD = 0  # torque value of phi for controller

w1 = 0  # angular for motor 1
w3 = 0  # angular for motor 3
x_velocity_accel = 0
y_velocity_accel = 0
z_velocity_accel = 0
x_distance_accel = 0
y_distance_accel = 0
z_distance_accel = 0

w_array = []

def cal_mpu():  # this is mpu calibration
    while True:
        try:
            accel_data = mpu.get_accel_data()
            gyro_data = mpu.get_gyro_data()
        except:
            continue

        mpu_accel_array.append([accel_data['x'], accel_data['y'], accel_data['z']])
        mpu_gyro_array.append([gyro_data['x'], gyro_data['y'], gyro_data['z']])

        if np.shape(mpu_accel_array)[0] == cal_size:

            for qq in range(0, 3):
                accel_offsets[qq] = np.mean(np.array(mpu_accel_array)[:, qq])
                accel_offsets[2] -= 9.80665
                gyro_offsets[qq] = np.mean(np.array(mpu_gyro_array)[:, qq ])
            break

    print('End Accel Cal ', accel_offsets)
    print('End Gyro Cal ', gyro_offsets)

    accel_data = mpu.get_accel_data()
    gyro_data = mpu.get_gyro_data()
    print('Data : ',accel_data['x']-accel_offsets[0], ' ', accel_data['y']-accel_offsets[1], ' ', accel_data['z']-accel_offsets[2])
    print('Data : ',gyro_data['x']-gyro_offsets[0], ' ', gyro_data['y']-gyro_offsets[1], ' ', gyro_data['z']-gyro_offsets[2])



def cal_dr_motors():  # this is motor calibration
    print(' Im starting the motor, calibrated and armed.....')
    time.sleep(1)
    speed0 = 1700
    speed1 = 1700
    print(' Controls to increase or decrease motor: a to increase, d to decrease')
    print(' q to increase a lot, e to decrease a lot')

    print('speed0 = ' + str(speed0) + '   speed1 = ' + str(speed1))
    pwm0.change_duty_cycle(speed0 / max_time_freq * 100)
    pwm1.change_duty_cycle(speed1 / max_time_freq * 100)


def control_man():  # this is manual control
    print(' Im starting the motor, calibrated and armed.....')
    time.sleep(1)
    speed0 = 1700
    speed1 = 1700
    print(' Controls to increase or decrease motor: a to increase, d to decrease')
    print(' q to increase a lot, e to decrease a lot')
    while True:
        if speed0 < 1700:
            speed0 = 1700
        elif speed0 > 2000:
            speed0 = 2000
        else:
            speed0 = speed0
        if speed1 < 1700:
            speed1 = 1700
        elif speed1 > 2000:
            speed1 = 2000
        else:
            speed1 = speed1

        print('speed0 = ' + str(speed0) + '   speed1 = ' + str(speed1))
        pwm0.change_duty_cycle(speed0 / max_time_freq * 100)
        pwm1.change_duty_cycle(speed1 / max_time_freq * 100)

        inp = input()
        if inp == "e":
            speed0 += com_val
            speed1 += com_val

        elif inp == "q":
            speed0 -= com_val
            speed1 -= com_val

        elif inp == "d":
            speed0 += sep_val
            speed1 -= sep_val

        elif inp == "a":
            speed0 -= sep_val
            speed1 += sep_val

        elif inp == "stop":
            stop()

        else:
            print("please type a(-1, +1), d(+1, -1), q(-10, -10), e(+10, +10), stop")

def pause():
    pwm0.change_duty_cycle(min_duty)
    pwm1.change_duty_cycle(min_duty)

def stop():
    pwm0.change_duty_cycle(0)
    pwm0.stop()
    pwm1.change_duty_cycle(0)
    pwm1.stop()
    exit()

def control_pid():  # this is pid control
    f = open('SingleArmData.csv', 'w', newline= '')
    write = csv.writer(f)
    write.writerow(['time', 'time_change', 'phi velocity', 'phi angle', 'thrust', 'torque', 'w1', 'w3', 'PWM w1', 'PWM w3'])
    w_ave = 1800
    w1 = w_ave  # angular for motor 1
    w3 = w_ave  # angular for motor 3
    w_tot = w_ave  # angular for motor 1
    w1_new = w_ave  # angular for motor 1
    w3_new = w_ave  # angular for motor 3
    x_velocity_accel = 0
    y_velocity_accel = 0
    z_velocity_accel = 0
    x_distance_accel = 0
    y_distance_accel = 0
    z_distance_accel = 0
    phi_gyro = 0.0
    theta_gyro = 0.0
    psi_gyro = 0.0
    phi_gyro_data_rad = 0.0
    theta_gyro_data_rad = 0.0
    psi_gyro_data_rad = 0.0
    phi_gyro_rad = 0.0
    theta_gyro_rad = 0.0
    psi_gyro_rad = 0.0
    pwm_loop = 1
    w_array = []
    time_change = 0
    T_PD = 0
    torq_phi_PD = 0
    last_time = time.time()
    time.sleep(5)
    print('phi_gyro = ', phi_gyro, 'theta_gyro = ', theta_gyro)
    print('w1 = ', w1, 'w3 = ', w3)
    print()

    while True :
            accel_data = mpu.get_accel_data()
            gyro_data = mpu.get_gyro_data()
            new_time = time.time()
            if(pwm_loop > 1 ):
                time_change = new_time - last_time

                x_velocity_accel += (accel_data['x'] - accel_offsets[0]) * (time_change)
                y_velocity_accel += (accel_data['y'] - accel_offsets[1]) * (time_change)
                z_velocity_accel += (accel_data['z'] - accel_offsets[2]) * (time_change)

                x_distance_accel += x_velocity_accel * time_change
                y_distance_accel += y_velocity_accel * time_change
                z_distance_accel += z_velocity_accel * time_change

                phi_gyro_data_rad = (gyro_data['x'] - gyro_offsets[0])*math.pi/180
                theta_gyro_data_rad = (gyro_data['y'] - gyro_offsets[1])*math.pi/180
                psi_gyro_data_rad = (gyro_data['z'] - gyro_offsets[2])*math.pi/180

                phi_gyro += (gyro_data['x'] - gyro_offsets[0]) * (time_change)
                theta_gyro += (gyro_data['y'] - gyro_offsets[1]) * (time_change)
                psi_gyro += (gyro_data['z'] - gyro_offsets[2]) * (time_change)

                phi_gyro_rad = phi_gyro*math.pi/180
                theta_gyro_rad = theta_gyro*math.pi/180
                psi_gyro_rad = psi_gyro*math.pi/180

                T_PD = g * m / math.cos(phi_gyro_rad) / math.cos(theta_gyro_rad)
                torq_phi_PD = (KphiD * (phi_v_d - phi_gyro_data_rad) + KphiP * (phi_d - phi_gyro_rad))*0.5
                #w3 = math.sqrt(T_PD / 2 / k+torq_phi_PD / 2 / k / l)
                #w1 = math.sqrt(T_PD / 2 / k - torq_phi_PD / 2 / k / l)

                print('phi velocity = ', gyro_data['x'] - gyro_offsets[0])
                print('phi velocity*time = ', (gyro_data['x'] - gyro_offsets[0])*time_change)
                print('phi_gyro = ', phi_gyro, 'theta_gyro = ', theta_gyro)
                print('thrust = ', T_PD, 'torque = ', torq_phi_PD)
                print('T_PD/2/k =',T_PD/2/k, 'torq_phi_PD/2/k/l = ', torq_phi_PD/2/k/l)
                print('timechange =', time_change)
                print('w1 = ', w1, 'w3 = ', w3)

               # write.writerow([new_time, time_change, gyro_data['x'] - gyro_offsets[0], phi_gyro, T_PD, torq_phi_PD, w1, w3, w1_pwm, w3_pwm])

                if  (T_PD >= torq_phi_PD/l) :
                    print ('Correct math')
                else :
                    print ('Error')
                    stop()
                print()
                w3 = math.sqrt(abs(T_PD / 2 / k+torq_phi_PD / 2 / k / l))
                w1 = math.sqrt(abs(T_PD / 2 / k - torq_phi_PD / 2 / k / l))

                #w_array.append([new_time, w1, w3])
            pwm_loop += 1

            w_tot = w1 +w3
            w1_pwm = 1744-50+100*w1/w_tot
            w3_pwm = 1856-50+100*w3/w_tot
            pwm0.change_duty_cycle(w3_pwm / max_time_freq * 100)
            pwm1.change_duty_cycle(w1_pwm / max_time_freq * 100)

            print('w1_pwm = ', w1_pwm, 'w3_pwm = ', w3_pwm)
            
            write.writerow([new_time, time_change, gyro_data['x'] - gyro_offsets[0], phi_gyro, T_PD, torq_phi_PD, w1, w3, w1_pwm, w3_pwm])

            #       time.sleep()
            last_time = new_time
            time.sleep(.01)
            if (pwm_loop > 200):
                pwm0.change_duty_cycle(0)
                pwm0.stop()          
                pwm1.change_duty_cycle(0)
                pwm1.stop()
                break
    f.close()

while True:
    print('Enter cal_mpu [mpu], cal_dr_motors [mot], control_man [man], control_pid [pid], or stop')
    inp = input()
    if inp == "mpu":
        cal_mpu()
    elif inp == "stop":
        stop()
    elif inp == "pid":
        control_pid()
    elif inp == "man":
        control_man()
    elif inp == "mot":
        cal_dr_motors()
    elif inp == "pause":
        pause()
    else:
        print('Enter cal_mpu [mpu], cal_dr_motors [mot], control_man [man], control_pid [pid], or stop')
