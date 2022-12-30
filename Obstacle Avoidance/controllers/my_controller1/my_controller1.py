from controller import Robot , GPS , Motor , Gyro , Camera, CameraRecognitionObject , InertialUnit, Compass , RangeFinder
import numpy
from numpy import *
import glob 
import cv2
from controller import Keyboard


robot = Robot()

timestep = 10

def CLAMP(value, low, high): 
    if value < low:
        return low
    elif value > high:
        return high
    else:
        return value

 

                



camera = robot.getDevice("camera")
camera.enable(timestep)
#camera.recognitionEnable(timestep)

# kinect = robot.getDevice("kinect range")
# kinect.enable(timestep)

lidar = robot.getDevice('lidar')
lidar.enable(timestep)


dist_r = robot.getDevice('dist_r')
dist_r.enable(timestep)


dist_front = robot.getDevice('dist_front')
dist_front.enable(timestep)


dist_l = robot.getDevice('dist_l')
dist_l.enable(timestep)


front_left_led = robot.getDevice("front left led")
front_right_led = robot.getDevice("front right led")

imu = robot.getDevice("inertial unit")
imu.enable(timestep)

gps = robot.getDevice("gps")
gps.enable(timestep)

compass = robot.getDevice("compass")
compass.enable(timestep)


gyro = robot.getDevice("gyro")
gyro.enable(timestep)

keyboard=Keyboard()
keyboard.enable(timestep)


camera_roll_motor = robot.getDevice("camera roll")
camera_pitch_motor = robot.getDevice("camera pitch")
camera_yaw_motor = robot.getDevice("camera yaw")

front_left_motor = robot.getDevice("front left propeller")
front_right_motor = robot.getDevice("front right propeller")
rear_left_motor = robot.getDevice("rear left propeller")
rear_right_motor = robot.getDevice("rear right propeller")

front_left_motor.setPosition(float("inf"))
front_left_motor.setVelocity(1.0)

front_right_motor.setPosition(float("inf"))
front_right_motor.setVelocity(1.0)

rear_left_motor.setPosition(float("inf"))
rear_left_motor.setVelocity(1.0)

rear_right_motor.setPosition(float("inf"))
rear_right_motor.setVelocity(1.0)

  #Display the welcome message.
print("Start the drone...\n")

  #Wait one second.
while robot.step(timestep) != -1: 
    if (robot.getTime() > 1.0):
        break
    
print("You can control the drone with your computer keyboard:\n");
print("- 'up': move forward.\n");
print("- 'down': move backward.\n");
print("- 'right': turn right.\n");
print("- 'left': turn left.\n");
print("- 'shift + up': increase the target altitude.\n");
print("- 'shift + down': decrease the target altitude.\n");
print("- 'shift + right': strafe right.\n");
print("- 'shift + left': strafe left.\n");

k_vertical_thrust = 68.5 
k_vertical_offset = 0.6   
k_vertical_p = 3.0        
k_roll_p = 50.0 
k_pitch_p = 30.0  
target_altitude = 1.0


width  = camera.getWidth()
height = camera.getHeight()

x_new = -25
y_new = 1.86747
z_new = -6.40985


while robot.step(timestep) != -1:

    gps_value = gps.getValues()
    #print('gps_value' , gps_value)
        
    gps_v0 = gps_value[0]
    gps_v1 = gps_value[1]
    gps_v2 = gps_value[2]
    

    #kinect_v = kinect.getRangeImage()
    #print('kinect_v' , kinect_v)
    #print('-----------------------------')
    lidar_v = lidar.getRangeImage()
    #print('lidar_v' , lidar_v)
    lidar_vl = lidar_v[0:2]
    lidar_vf = lidar_v[2:4]
    lidar_vr = lidar_v[4:6]
    #print('lidar_vl' , lidar_vl)
    #print('lidar_vf' , lidar_vf)
    #print('lidar_vr' , lidar_vr)
    
    
    
    
    
    dist_v_l = dist_l.getValue()
    # print('dist_v_l' , dist_v_l)
    
    dist_v_f = dist_front.getValue()
    # print('dist_v_f' , dist_v_f)
    
    dist_v_r = dist_r.getValue()
    # print('dist_v_r' , dist_v_r)
    

    roll = imu.getRollPitchYaw()[0] + numpy.pi / 2.0
    pitch = imu.getRollPitchYaw()[1]
    altitude = gps.getValues()[1]
    roll_acceleration = gyro.getValues()[0]
    pitch_acceleration = gyro.getValues()[1]
    
    camera_roll_motor.setPosition( -0.115 * roll_acceleration)
    camera_pitch_motor.setPosition(-0.1 * pitch_acceleration)
    
    roll_disturbance = 0.0
    pitch_disturbance = 0.0
    yaw_disturbance = 0.0

    key = keyboard.getKey()
   
    if (key) : 
        if (key == Keyboard.UP) :
            pitch_disturbance = 2.0;
        elif(key == Keyboard.DOWN) : 
            pitch_disturbance = -2.0
        elif(key == Keyboard.RIGHT) : 
             yaw_disturbance = 1.3
        elif(key == Keyboard.LEFT) :
             yaw_disturbance = -1.3
        elif(key == Keyboard.SHIFT + Keyboard.RIGHT) : 
             roll_disturbance = -1.0
        elif(key == Keyboard.SHIFT + Keyboard.LEFT) :
            roll_disturbance = 1.0
        elif(key == Keyboard.SHIFT + Keyboard.UP) :
            target_altitude += 0.05
            print("target altitude: %f [m]\n", target_altitude)
        elif(key == Keyboard.SHIFT + Keyboard.DOWN) :
            target_altitude -= 0.05
            print("target altitude: %f [m]\n", target_altitude)

        
    # for a in lidar_vf :
                # for b in lidar_vr :
                    # for c in lidar_vl :
                        # if x_new < -0.054471 and gps_v0 > x_new : 
                            # if a < 10 :
                                # if b < 10:
                                    # roll_disturbance = 1.0
                                
                                
                                # else :
                                    # roll_disturbance = 1.0
                        
                            # elif b < 10 :
                                # roll_disturbance = 1.0 
                            # elif c < 10 :
                                # roll_disturbance = 1.0
                            # pitch_disturbance = 1

    if x_new < -0.054471 and gps_v0 > x_new : 
            for a in lidar_vf :
                        for b in lidar_vr :
                            for c in lidar_vl :
                                if a < 10 :
                                        if b < 10:
                                            roll_disturbance = 1.0
                                        
                                        
                                        else :
                                            roll_disturbance = 1.0
                                
                                elif b < 10 :
                                    roll_disturbance = 1.0 
                                elif c < 10 :
                                    roll_disturbance = 1.0
            pitch_disturbance = 1           
    # if dist_v_f < 980 :
        # if dist_v_r < 980 :
            # roll_disturbance = 1.0
        # else :
            # roll_disturbance = -1.0
    # elif dist_v_r < 980 :
            # roll_disturbance = 1.0
    # elif  dist_v_l < 980 :
            # roll_disturbance = -1.0            

              
    # if x_new < -0.054471 and gps_v0 > x_new :
        # for x in lidar_vf :
            # if x < 3 :
                # for y in lidar_vr :
                    # if y < 3 :
                        # roll_disturbance = 1.0
                    # else :
                        # roll_disturbance = -1.0
            # elif :
                # for y in lidar_vr : 
                    # if y < 3 :
                        # roll_disturbance = 1.0
            # elif :
                # for z in lidar_vl :
                    # if z < 3 :
                        # roll_disturbance = -1.0     
   
        # pitch_disturbance = 1
        
    # elif x_new > -0.054471 and  gps_v0 < x_new :
        # if dist_v_f < 980 :
            # if dist_v_r < 980 :
                # roll_disturbance = 1.0
            # else :
                # roll_disturbance = -1.0
        # elif dist_v_r < 980 :
                # roll_disturbance = 1.0
        # elif  dist_v_l < 980 :
                # roll_disturbance = -1.0 
        # pitch_disturbance = -0.3 
        
    # if y_new > 1.39874 and  gps_v1 < y_new :
        # roll_disturbance = -1.0            
    # elif y_new < 1.39874 and  gps_v1 > y_new :
        # roll_disturbance = -1.0 
                  
    # if z_new > -0.0568261 and  gps_v2 < z_new :
        # yaw_disturbance = -1.3          
    # elif z_new < -0.0568261 and  gps_v2 > z_new :
        # yaw_disturbance =  1.3           
                    
                                                 
   
    
    roll_input = k_roll_p * CLAMP(roll, -1.0, 1.0) + roll_acceleration + roll_disturbance
    pitch_input = k_pitch_p * CLAMP(pitch, -1.0, 1.0) - pitch_acceleration + pitch_disturbance
    yaw_input = yaw_disturbance;
    clamped_difference_altitude = CLAMP(target_altitude - altitude + k_vertical_offset, -1.0, 1.0)
    vertical_input = k_vertical_p * pow(clamped_difference_altitude, 3.0)

   
    front_left_motor_input = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input
    front_right_motor_input = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input
    rear_left_motor_input = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input
    rear_right_motor_input = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input
    
    
    front_left_motor.setVelocity(front_left_motor_input)

    front_right_motor.setVelocity(-front_right_motor_input)

    rear_left_motor.setVelocity(-rear_left_motor_input)

    rear_right_motor.setVelocity(rear_right_motor_input)



