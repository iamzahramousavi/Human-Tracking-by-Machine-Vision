from controller import *
import mavic2proHelper
from simple_pid import PID
import csv
import struct
import math
import matplotlib.pyplot as plt
import numpy as np

params = dict()
with open("../params.csv", "r") as f:
	lines = csv.reader(f)
	for line in lines:
		params[line[0]] = line[1]

TIME_STEP = int(params["QUADCOPTER_TIME_STEP"])
TAKEOFF_THRESHOLD_VELOCITY = int(params["TAKEOFF_THRESHOLD_VELOCITY"])
M_PI = 3.1415926535897932384626433

robot = Robot()

[frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor] = mavic2proHelper.getMotorAll(robot)

timestep = int(robot.getBasicTimeStep())
mavic2proMotors = mavic2proHelper.getMotorAll(robot)
mavic2proHelper.initialiseMotors(robot, 0)
mavic2proHelper.motorsSpeed(robot, TAKEOFF_THRESHOLD_VELOCITY, TAKEOFF_THRESHOLD_VELOCITY, TAKEOFF_THRESHOLD_VELOCITY, TAKEOFF_THRESHOLD_VELOCITY)

front_left_led =  robot.getDevice("front left led")
front_right_led =  robot.getDevice("front right led")

gps = robot.getDevice('gps')
gps.enable(TIME_STEP)
imu =  robot.getDevice("inertial unit")
imu.enable(TIME_STEP)
compass =  robot.getDevice("compass")
compass.enable(TIME_STEP)
gyro =  robot.getDevice("gyro")
gyro.enable(TIME_STEP)

camera =  robot.getDevice("camera")
camera.enable(TIME_STEP)
arrow1 =  robot.getDevice("arrow")
arrow1.setPosition(float('inf'))

compass2 = robot.getDevice('compass2')
compass2.enable(TIME_STEP)

yaw_setpoint=-1

pitchPID = PID(float(params["pitch_Kp"]), float(params["pitch_Ki"]), float(params["pitch_Kd"]), setpoint=0.0)
rollPID = PID(float(params["roll_Kp"]), float(params["roll_Ki"]), float(params["roll_Kd"]), setpoint=0.0)
throttlePID = PID(float(params["throttle_Kp"]), float(params["throttle_Ki"]), float(params["throttle_Kd"]), setpoint=1)
yawPID = PID(float(params["yaw_Kp"]), float(params["yaw_Ki"]), float(params["yaw_Kd"]), setpoint=float(yaw_setpoint))

targetX, targetY, target_altitude = 0.0, 0.0, 1.0

listTarget1=[1.0, 0.0, 1.0]
listTarget2=[1.0, 1.0, 1.0]
listTarget3=[2.0, 1.0, 1.0]
total = [[-1.0, 0.27, 1.0],[-1.41, 0.6, 1.0], [-1.73, 1.0, 1.0], [-2.0, 2.0, 1.0], [-1.73, 3.0, 1.0],[-1.41, 3.41, 1.0], [-1.0, 3.73, 1.0],[0.0, 4.0, 1.0], [1.0, 3.73, 1.0],[1.41, 3.41, 1.0], [1.73, 3.0, 1.0], [2.0, 2.0, 1.0], [1.73, 1.0, 1.0],[1.41, 0.6, 1.0], [1.0, 0.27, 1.0],[0.0, 0.0, 1.0]]# newX = [1.0, 1.0, 2.0]
pedestrian = [ -2 , 1.27 , 0 ]

def runMotor():
     roll_input = float(params["k_roll_p"]) * roll + roll_acceleration + rollPID(xGPS)
     pitch_input = float(params["k_pitch_p"]) * pitch - pitch_acceleration + pitchPID(-yGPS)
                
     front_left_motor_input = float(params["k_vertical_thrust"]) + vertical_input - roll_input - pitch_input + yaw_input
     front_right_motor_input = float(params["k_vertical_thrust"]) + vertical_input + roll_input - pitch_input - yaw_input
     rear_left_motor_input = float(params["k_vertical_thrust"]) + vertical_input - roll_input + pitch_input - yaw_input
     rear_right_motor_input = float(params["k_vertical_thrust"]) + vertical_input + roll_input + pitch_input + yaw_input
     return front_left_motor_input, front_right_motor_input, rear_left_motor_input, rear_right_motor_input     


R = 2
max_theta = 2* np.pi
list_t = list((np.arange(4.71, -4.71 , -0.17) ))

  
x_circle = [round((R*math.cos(x_y)) , 2) for x_y  in list_t]
y_circle = [round((R*math.sin(x_y)) , 2) for x_y  in list_t ]
new_y = []


for i in y_circle :
    if i < 0 :
        new_y.append(R - abs(i))
    else :
        new_y.append( R + i)    
          	
# print('list_t' , list_t)
# print('x_circle' , x_circle)
# print('y_circle' , y_circle) 
# print('new_y' , new_y) 







state = 0
while (robot.step(timestep) != -1):
    led_state = int(robot.getTime()) % 2
    front_left_led.set(led_state)
    front_right_led.set(int(not(led_state)))
        
    roll = imu.getRollPitchYaw()[0] + M_PI / 2.0
    pitch = imu.getRollPitchYaw()[1]
    yaw = compass.getValues()[0]
    roll_acceleration = gyro.getValues()[0]
    pitch_acceleration = gyro.getValues()[1]
        	
    xGPS = gps.getValues()[2]
    yGPS = gps.getValues()[0]
    zGPS = gps.getValues()[1]
        
    vertical_input = throttlePID(zGPS)
    yaw_input = yawPID(yaw)
        
       

            
    for i in range(len(x_circle)):
        if state == i :
    
            # #Top Right
            if x_circle[i] < 0.0 and new_y[i]  >= 0.0: 
                dx = abs(x_circle[i] ) - abs(xGPS)
                dy= new_y[i] -  abs(yGPS)
                dis = math.sqrt(dx**2 + dy**2)
                # print('dx ',dx)
                # print('dy ', dy)
                # print('dis ', dis)
              
            #Top Left
            elif x_circle[i] >= 0.0 and new_y[i] >= 0.0: 
                dx = x_circle[i] - abs(xGPS)
                dy= new_y[i] -  abs(yGPS)
                dis = math.sqrt(dx**2 + dy**2)
                # print('dx ',dx)
                # print('dy ', dy)
                # print('dis ', dis)
                
            #Down Right
            elif x_circle[i] < 0.0 and new_y[i] < 0.0: 
                dx = abs(x_circle[i]) - abs(xGPS)
                dy= abs(new_y[i]) -  abs(yGPS)
                dis = math.sqrt(dx**2 + dy**2)
                # print('dx ',dx)
                # print('dy ', dy)
                # print('dis ', dis)
              
            #Down Left
            elif x_circle[i] >= 0.0 and new_y[i] < 0.0: 
                dx = x_circle[i] - abs(xGPS)
                dy= abs(new_y[i]) -  abs(yGPS)
                dis = math.sqrt(dx**2 + dy**2)
                # print('dx ',dx)
                # print('dy ', dy)
                # print('dis ', dis)
                         
            rollPID.setpoint = x_circle[i]
            pitchPID.setpoint = new_y[i]
                
            fl , fr , rl , rr = runMotor()
            
            mavic2proHelper.motorsSpeed(robot, fl, -fr, -rl, rr)
        
            if state == (len(x_circle) - 1):
                break
    
            if dis < 0.2 :
                state+=1
      
    # print('state ',state)        
    # print('--------------')
    
    
         
    com1 = compass.getValues()
    angle = math.atan2(com1[0], com1[2]) 
    if (angle > 0.0) :
         angle = angle + 3.14
    print('angle',angle)

         
    dx= xGPS-pedestrian[2]
    dy= yGPS-pedestrian[0]
    dt= math.atan2(dx,dy)
    if (dt > 0.0) :
         dt = dt - 6.28
    if dx < 0 :
        n2 = 1.64
    else :
        n2 = 1.57    
    do = ((angle - dt) + n2 )
    
    #####
    com2 = compass2.getValues()
    zavie1=math.atan2(com2[0],com2[2])
    bearing = (zavie1 - 1.5708) / math.pi * 180.0
    if (bearing < 0.0) :
         bearing = bearing + 360.0 
    
    dy2= xGPS-pedestrian[2]
    dx2= yGPS-pedestrian[0]
    dt2= 360-(math.atan2(dy,dx)*180/math.pi)
    do2 = bearing-dt2
    
    if abs(do2)>180:
        dop=360-(-1*do2) 
        do2=dop   
    # print("bearing", bearing)
    # print('dt2', dt2)
    # print("do2", do2)
    if do2 > 2 :
        arrow1.setVelocity(2.5) 
    elif do2< -2 :
        arrow1.setVelocity(-2.5) 
    else :
        arrow1.setVelocity(0) 
    ####
    # print("n2", n2)
    
    # print("xGPS", xGPS)
    # print("yGPS", yGPS)
    # print("do", do)
    # print('dt', dt)
    #arrow1.setPosition(do) 
    # print(".............")
    