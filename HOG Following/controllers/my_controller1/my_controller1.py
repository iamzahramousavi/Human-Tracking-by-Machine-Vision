from controller import Robot , GPS , Motor , Gyro , Camera, CameraRecognitionObject , InertialUnit, Compass , RangeFinder
import numpy
from numpy import *
import numpy as np
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






def click_event(event, x, y, flags, params):
 
    # checking for left mouse clicks
    if event == cv2.EVENT_LBUTTONDOWN:
 
        # displaying the coordinates
        # on the Shell
        print(x, ' ', y)
 
        # displaying the coordinates
        # on the image window
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(frame, str(x) + ',' +
                    str(y), (x,y), font,
                    1, (255, 0, 0), 2)
        cv2.imshow('frame', frame)
 
    # checking for right mouse clicks    
    if event==cv2.EVENT_RBUTTONDOWN:
 
        # displaying the coordinates
        # on the Shell
        print(x, ' ', y)
 
        # displaying the coordinates
        # on the image window
        font = cv2.FONT_HERSHEY_SIMPLEX
        b = img[y, x, 0]
        g = img[y, x, 1]
        r = img[y, x, 2]
        cv2.putText(img, str(b) + ',' +
                    str(g) + ',' + str(r),
                    (x,y), font, 1,
                    (255, 255, 0), 2)
        cv2.imshow('frame', frame)

rightdetect = False
def HD(lidar_vf , lidar_vr , lidar_vl ,xC ) :                                    
    for a in lidar_vf :
        for b in lidar_vr :
            for c in lidar_vl :            
                if  xC[1] < 360  :
                    if  xC[1] < 165:
                        rightdetect = True
                        return rightdetect
                    elif xC[1] > 175:
                        rightdetect = False
                        return rightdetect
                # elif boxNum[1] < 360 :
                     # detect = True   
                
    
               



left = False
def OD(lidar_vf , lidar_vr , lidar_vl):
    for a in lidar_vf :
                for b in lidar_vr :
                    for c in lidar_vl :
                        #if x_new < -0.054471 and gps_v0 > x_new : 
                            if a < 10 :
                                if b < 10 and boxNum[1] == none:
                                    left = True
                                    return left
                                    # roll_disturbance = 1.0
                                else  :
                                    left = False
                                    return left
                                    # roll_disturbance = -1.0
                        
                            elif b < 10 and boxNum[1] == none:
                                left = True
                                return left
                                # roll_disturbance = 1.0 
                            elif c < 10 and boxNum[1] == none :
                                left = False
                                return left
                                # roll_disturbance = -1.0
    





# def humanTracking()
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
# cv2.startWindowThread()
while robot.step(timestep) != -1:
    
    
    frame = camera.getImage()
    frame = np.frombuffer(frame, np.uint8).reshape((height, width, 4))
    frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
   
    boxes, weights = hog.detectMultiScale(frame, winStride=(4,4))
    boxes = np.array([[x, y, x + w, y + h] for (x, y, w, h) in boxes])
    
    
        
        
        
        
        
    gps_value = gps.getValues()
    #print('gps_value' , gps_value)
        
    gps_v0 = gps_value[0]
    gps_v1 = gps_value[1]
    gps_v2 = gps_value[2]
    

    #kinect_v = kinect.getRangeImage()
    #print('kinect_v' , kinect_v)
    #print('-----------------------------')
    lidar_v = lidar.getRangeImage()
    # #print('lidar_v' , lidar_v)
    lidar_vl = lidar_v[0:2]
    lidar_vf = lidar_v[2:4]
    lidar_vr = lidar_v[4:6]
    # print('lidar_vl' , lidar_vl)
    # print('lidar_vf' , lidar_vf)
    # print('lidar_vr' , lidar_vr)
    
    
    
    
    
    # dist_v_l = dist_l.getValue()
    # print('dist_v_l' , dist_v_l)
    
    # dist_v_f = dist_front.getValue()
    # print('dist_v_f' , dist_v_f)
    
    # dist_v_r = dist_r.getValue()
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
            pitch_disturbance = 2.0
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
    
    
    
    
    
    
   
    boxNumA = [0]
    boxNumB = [0]
    xC = [0]
    for (xA, yA, xB, yB) in boxes:
        #display the detected boxes in the colour picture
        cv2.rectangle(frame, (xA, yA), (xB, yB),
                          (0, 255, 0), 2)
        boxNumA.append(xA)
        boxNumB.append(xB)
        # a = boxNum[1]
        # print(boxNum[1])
        xC.append(( boxNumA[1] +  boxNumB[1])/2)
        if HD(lidar_vf , lidar_vr , lidar_vl ,xC ):
            roll_disturbance = 1.0
            pitch_disturbance = 2.0
        elif HD(lidar_vf , lidar_vr , lidar_vl ,xC ) == False:
            roll_disturbance = -1.0 
            pitch_disturbance = 2.0                
        else :
            pitch_disturbance = 2.0
            
         

               
      

    
  
    #Display the resulting frame
    cv2.imshow('frame',frame)
    cv2.setMouseCallback('frame', click_event)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
        
             
    
        # pitch_disturbance = 2.0          
   
    
        
                   
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            

        
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

    # if x_new < -0.054471 and gps_v0 > x_new : 
            # for a in lidar_vf :
                        # for b in lidar_vr :
                            # for c in lidar_vl :
                                # if a < 10 :
                                        # if b < 10:
                                            # roll_disturbance = 1.0
                                        
                                        
                                        # else :
                                            # roll_disturbance = -1.0
                                
                                # elif b < 10 :
                                    # roll_disturbance = 1.0 
                                # elif c < 10 :
                                    # roll_disturbance = -1.0
            # pitch_disturbance = 1
                       
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



