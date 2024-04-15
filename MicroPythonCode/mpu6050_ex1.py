from imu import MPU6050
import time
from machine import Pin, I2C
from vector3d import Vector3d
import math

i2c = I2C(0, sda=Pin(16), scl=Pin(17), freq=400000)
imu = MPU6050(i2c)
accel_v = [0,0,0]
accel_x = [0,0,.3]
gyro_ang = [0,0,0]
last_time = 0
gravity = 9.80665

def mpu_cal(calval):
    print("accel range", imu.accel_range, "\t", "gyro range", imu.gyro_range)
    print("accel pre offset ", imu.accel.x," ",imu.accel.y," ",imu.accel.z, "\t", "gyro pre offset ", imu.gyro.x," ",imu.gyro.y," ",imu.gyro.z,)
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

    print("accel offset", accel_offset, "gyro offset", gyro_offset)
    time.sleep(2)
    last = time.ticks_ms()

    #for i in range (1,2):
        #delta = time.ticks_diff(time.ticks_ms(), last)
        #last = time.ticks_ms()
        #accel = [imu.accel.x-accel_offset[0],imu.accel.y-accel_offset[1],imu.accel.z-accel_offset[2]]
        #gyro = [imu.gyro.x-gyro_offset[0],imu.gyro.y-gyro_offset[1],imu.gyro.z-gyro_offset[2]]
        #accel_v =[accel_v[i]+ accel[i]*gravity*delta*0.001 for i in range(3)]
        #accel_x =[accel_x[i]+ accel_v[i]*delta*0.001 for i in range(3)]
        #gyro_ang = [gyro_ang[i]+gyro[i]*delta*0.001 for i in range(3)]
        #accel_v[0] += accel[0]*gravity*delta*0.001
        #accel_v[1] += accel[1]*gravity*delta*0.001
        #accel_v[2] += accel[2]*gravity*delta*0.001
        #accel_x[0] += accel_v[0]*delta*0.001
        #accel_x[1] += accel_v[1]*delta*0.001
        #accel_x[2] += accel_v[2]*delta*0.001
        #gyro_ang[0] += gyro[0]*delta*0.001
        #gyro_ang[1] += gyro[1]*delta*0.001
        #gyro_ang[2] += gyro[2]*delta*0.001
        #print("accel", accel, "gyro", gyro)
        #print("accel_v", accel_v, "delta", delta)
        #print("accel_x", accel_x, "delta", delta)
        #print("gyro_ang", gyro_ang, "delta", delta)
    
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
    #accel_v[0] += accel[0]*gravity*delta*0.001
    #accel_v[1] += accel[1]*gravity*delta*0.001
    #accel_v[2] += accel[2]*gravity*delta*0.001
    #accel_x[0] += accel_v[0]*delta*0.001
    #accel_x[1] += accel_v[1]*delta*0.001
    #accel_x[2] += accel_v[2]*delta*0.001
    #gyro_ang[0] += gyro[0]*delta*0.001
    #gyro_ang[1] += gyro[1]*delta*0.001
    #gyro_ang[2] += gyro[2]*delta*0.001
    print("accel", accel, "gyro", gyro)
    print("accel_v", accel_v, "delta", delta)
    print("accel_x", accel_x, "delta", delta)
    print("gyro_ang", gyro_ang, "delta", delta)
    return accel, accel_v, accel_x, gyro, gyro_ang, last_time

def control_pd(accel_offset, gyro_offset):
    gravity = 9.80665
    mass = 1.1
    l = 0.254    #length of drone arm
    k = 2.980e-6 #lift constant
    b = 1.140e-7 #drag constant
    II = [0.01,0.01,0.01]
    accel_v = [0.0,0.0,0.0]
    accel_x = [0.0,0.0,0.0]
    gyro_ang = [0.0,0.0,0.0]
    
    accel_vd = [0.0,0.0,0.0]
    accel_xd = [0.0,0.0,0.0]
    gyro_d = [0.0,0.0,0.0]
    gyro_angd = [0.0,0.0,0.0]
    
    
    K_accel_D = [1.5,1.5,1.5]
    K_accel_P = [2.0,2.0,2.0]
    #K_accel_I = [0.5,0.5,0.5]
    K_gyro_D = [2.0,2.0,2.0]
    K_gyro_P = [6.0,6.0,6.0]
    #K_gyro_I = [1.5,1.5,1.5]
    
    last_time = time.ticks_ms()
    for j in range (1,100):
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
        gyro_ang_rad[2] = 0.0 # no update for psi angle
        
        #T_PD = (gravity + K_accel_D[2]*(accel_vd[2] - accel_v[2]) + K_accel_P[2]*(accel_xd[2] - accel_x[2]))* \
        #      mass/math.cos(gyro_ang_rad[0])/math.cos(gyro_ang_rad[1])
        
        T_PD = (gravity)*mass/math.cos(gyro_ang_rad[0])/math.cos(gyro_ang_rad[1])
        
        torq_PD = [(K_gyro_D[i]*(gyro_d[i]-gyro_rad[i])+K_gyro_P[i]*(gyro_angd[i]-gyro_ang_rad[i]))*II[i] for i in range(3)]
        
        print("T_PD :  ", T_PD)
        print("T_PD elements", accel_vd[2] - accel_v[2], accel_xd[2] - accel_x[2], math.cos(gyro_ang_rad[0]), math.cos(gyro_ang_rad[1]))
        print("torq PD, phi, theta,psi: ", torq_PD)
        print("torq_PD elements phi: ", gyro_d[0]-gyro[0], gyro_angd[0]-gyro_ang[0])
        print("torq_PD elements theta: ", gyro_d[1]-gyro[1], gyro_angd[1]-gyro_ang[1])
        print("torq_PD elements psi: ", gyro_d[2]-gyro[2], gyro_angd[2]-gyro_ang[2])
        
        
        w1 = math.sqrt(math.fabs(T_PD/4/k-torq_PD[0]/2/k/l+torq_PD[2]/2/b))
        w0 = math.sqrt(math.fabs(T_PD/4/k-torq_PD[1]/2/k/l-torq_PD[2]/2/b))
        w3 = math.sqrt(math.fabs(T_PD/4/k+torq_PD[0]/2/k/l+torq_PD[2]/2/b))
        w2 = math.sqrt(math.fabs(T_PD/4/k+torq_PD[1]/2/k/l-torq_PD[2]/2/b))
        
        print(" w elements: ", T_PD/4/k, torq_PD[0]/2/k/l, torq_PD[1]/2/k/l, torq_PD[2]/2/b)
        print(" w0-3 : ", w0, w1, w2, w3)
        print(" pwm0-3 : ", w0*1.5, w1*1.4, w2*1.1, w3*1.4)
        print('')
    
        
        time.sleep(0.005)
        
    return w0,w1,w2,w3

mpu_offsets = mpu_cal(10)
control_pd(mpu_offsets[0],mpu_offsets[1])

print('')

print("accel_offsets", mpu_offsets[0], "gyro_offsets", mpu_offsets[1])
last_time = time.ticks_ms()
#for i in range (1,50):
    #imu_outputs = mpu_outputs(mpu_offsets[0], mpu_offsets[1],accel_v,accel_x,gyro_ang, last_time)
    #accel_v = imu_outputs[1]
    #accel_x =imu_outputs[2]
    #gyro_ang = imu_outputs[4]
    #last_time = imu_outputs[5]
    #time.sleep(.01)
    #print('')