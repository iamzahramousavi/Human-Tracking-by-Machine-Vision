from controller import Robot , GPS , Motor , Gyro , Camera, CameraRecognitionObject , InertialUnit, Compass , RangeFinder
import numpy
from numpy import *
import numpy as np
import glob 
import cv2
from controller import Keyboard
import math

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
camera.recognitionEnable(timestep)


# kinect = robot.getDevice("kinect range")
# kinect.enable(timestep)

lidar = robot.getDevice('lidar')
lidar.enable(timestep)
lidar.enablePointCloud()


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
forwarddetect = False
def HD(lidar_vf , lidar_vr , lidar_vl ,posA ) : 
     if  posA[1] < 160:
         rightdetect = False
         forwarddetect = False
         return rightdetect, forwarddetect
     elif posA[1] > 200:
          rightdetect = True
          forwarddetect = False
          return rightdetect, forwarddetect                                              
     else:
         rightdetect = False
         forwarddetect = True
         return rightdetect, forwarddetect    
        
        
                        
                        
                        
                        
                                   
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
    

max = 5
def lidrarR(lidar_vr ) :
            
    LidarR0 = []
    for i in (lidar_vr[0:1]) :
        if i == inf : 
            LidarR0.append(max)
        else :
            LidarR0.append(i)    
                # print('LidarR',LidarR)     
            
    LidarR0 = np.array(LidarR0)    
    minLidarR0 = np.min(LidarR0)    
        
    LidarR1 = []
    for i in (lidar_vr[1:2]) :
        if i == inf : 
            LidarR1.append(max)
        else :
            LidarR1.append(i)  
    LidarR1 = np.array(LidarR1)    
    minLidarR1 = np.min(LidarR1)
    
    LidarR2 = []
    for i in (lidar_vr[2:3]):
        if i == inf : 
            LidarR2.append(max)
        else :
            LidarR2.append(i)  
    LidarR2 = np.array(LidarR2)    
    minLidarR2 = np.min(LidarR2)
    
    
    return minLidarR0 , minLidarR1 , minLidarR2
    
def lidrarF(lidar_vf ) :             
    # LidarF = []
    # for i in (lidar_vf) :
   
        # LidarF.append(i)
         
    # LidarF = np.array( LidarF)    
    # minLidarF = np.min( LidarF) 
    
    
    LidarF0 = []
    for i in (lidar_vf[0:1]) :
        if i == inf : 
            LidarF0.append(max)
        else :
            LidarF0.append(i)  
    LidarF0 = np.array( LidarF0)    
    minLidarF0 = np.min( LidarF0)
    
    
    
    LidarF1 = []
    for i in (lidar_vf[1:3]) :
        if i == inf : 
            LidarF1.append(max)
        else :
            LidarF1.append(i)  
    LidarF1 = np.array( LidarF1)    
    minLidarF1 = np.min( LidarF1)
    
    
    
    LidarF2 = []
    for i in (lidar_vf[3:4]) :
        if i == inf : 
            LidarF2.append(max)
        else :
            LidarF2.append(i)  
    LidarF2 = np.array( LidarF2)    
    minLidarF2 = np.min( LidarF2)
    
    return minLidarF0 , minLidarF1 , minLidarF2
    
    
    
    # LidarR0 = []
    # for i in (lidar_vr[0:1]) :
        # if i != inf :
        # LidarR0.append(i)
            # print('LidarF',LidarF)     
        # print('totalLidar' , totalLidar)
    # LidarR0 = np.array( LidarR0)    
    # minLidarR0 = np.min( LidarR0)
    
    
    
    # LidarL0 = []
    # for i in (lidar_vl[0:1]) :
        # if i != inf :
        # LidarL0.append(i)
            # print('LidarF',LidarF)     
        # print('totalLidar' , totalLidar)
    # LidarL0 = np.array( LidarL0)    
    # minLidarL0 = np.min( LidarL0)
    
    

        
def lidrarL(lidar_vl ) :    
    LidarL0 = []
    for i in (lidar_vl[0:1]) :
        if i == inf : 
            LidarL0.append(max)
        else :
            LidarL0.append(i)  
    LidarL0 = np.array(LidarL0)    
    minLidarL0 = np.min(LidarL0)
    
    LidarL1 = []
    for i in (lidar_vl[1:2]) :
        if i == inf : 
            LidarL1.append(max)
        else :
            LidarL1.append(i)  
    LidarL1 = np.array(LidarL1)    
    minLidarL1 = np.min(LidarL1)          
    
    LidarL2 = []
    for i in (lidar_vl[2:3]) :
        if i == inf : 
            LidarL2.append(max)
        else :
            LidarL2.append(i)  
    LidarL2 = np.array(LidarL2)    
    minLidarL2 = np.min(LidarL2)          
              
    
    return minLidarL0 , minLidarL1 , minLidarL2


waypoints = [0]
distance = []
def waypoint(gps_v0 , resultF1 ) :
    count = 0
    for i in range(len(waypoints)):
        count =+ 1
        if count % 100 == 0 :
            waypoints.append(resultF1 + abs(gps_v0))
    maxWaypoints = np.max(np.array(waypoints)) 
            
    return maxWaypoints






# def humanTracking()
# hog = cv2.HOGDescriptor()
# hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
# cv2.startWindowThread()
while robot.step(timestep) != -1:
    
    
    # frame = camera.getImage()
    # frame = np.frombuffer(frame, np.uint8).reshape((height, width, 4))
    # frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
   
    # boxes, weights = hog.detectMultiScale(frame, winStride=(4,4))
    # boxes = np.array([[x, y, x + w, y + h] for (x, y, w, h) in boxes])
    
    
        
        
           
       
    gps_value = gps.getValues()
    # print('gps_value' , gps_value)
        
    gps_v0 = gps_value[0]
    gps_v1 = gps_value[1]
    gps_v2 = gps_value[2]
    # print("gps_v1", gps_v1)
    

    #kinect_v = kinect.getRangeImage()
    #print('kinect_v' , kinect_v)
    #print('-----------------------------')
    lidar_v = lidar.getRangeImage()
    
    # pointcloud = lidar.getPointCloud()
    
    # plist = []
    # for i in pointcloud:
        # plist.append(pointcloud[i])
        # print('plist',plist)
    
    
    
    # print('lidar_v' , lidar_v)
    # print('pointcloud',pointcloud)
    # print(type(pointcloud))
    lidar_vr = lidar_v[0:3]
    
    lidar_vf = lidar_v[3:7]
    
    lidar_vl = lidar_v[7:10]
    
    # print('lidar_vl' , lidar_vl)
    # print('lidar_vf' , lidar_vf)
    # print('lidar_vl0' , lidar_vl[0])
    # print('lidar_vl1' , lidar_vl[1])
    # print('lidar_vl2' , lidar_vl[2])
    # resultR , resultF, resultL  = lidrarHD(lidar_vr , lidar_vf , lidar_vl )
    # print("resultR", resultR)
    # print("resultF", resultF)
    # print("resultL", resultL)
    
    # resultR , resultF, resultL, resultF0, resultF2, resultF1, resultL0, resultR0   = lidrarHD(lidar_vr , lidar_vf , lidar_vl) 
    # print('resultL0' , resultL0)
    
    
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
    
    
    
    
    
    
   
    # boxNumA = [0]
    # boxNumB = [0]
    # xC = [0]
    # for (xA, yA, xB, yB) in boxes:
        # #display the detected boxes in the colour picture
        # cv2.rectangle(frame, (xA, yA), (xB, yB),
                          # (0, 255, 0), 2)
        # boxNumA.append(xA)
        # boxNumB.append(xB)
        # a = boxNum[1]
        # print(boxNum[1])
        # xC.append(( boxNumA[1] +  boxNumB[1])/2)
        # if HD(lidar_vf , lidar_vr , lidar_vl ,xC ):
            # roll_disturbance = 1.0
            # pitch_disturbance = 2.0
        # elif HD(lidar_vf , lidar_vr , lidar_vl ,xC ) == False:
            # roll_disturbance = -1.0 
            # pitch_disturbance = 2.0                
        # else :
            # pitch_disturbance = 2.0
            
         

               
      

    
  
    # #Display the resulting frame
    # cv2.imshow('frame',frame)
    # cv2.setMouseCallback('frame', click_event)
    # if cv2.waitKey(1) & 0xFF == ord('q'):
        # break
      

    

    
                 
                        
   
    resultL0 , resultL1 , resultL2= lidrarL(lidar_vl ) 
    resultR0 , resultR1 , resultR2= lidrarR(lidar_vr)
    resultF0 , resultF1 , resultF2= lidrarL(lidar_vf )             
    waypoints = waypoint(gps_v2 , resultF1)
                    
    print('waypoints' , waypoints)
    posA = [0] 
    posB = [0]
    sizeA = [0] 
    sizeB = [0]             
    number_of_objects = camera.getRecognitionNumberOfObjects()
    objects =  camera.getRecognitionObjects()
       # print("Objects", objects)
    encoding = 'utf-8'
    aa = 'pedestrian'   
    if    gps_v1 > 0.9:
              
            for i in range(number_of_objects) :
                pos = objects[i].get_position_on_image()
                size = objects[i].get_size_on_image()
                model = objects[i].get_model()
                 
                                    
                if model.decode(encoding) == aa :
                    posA.append(pos[0])
                    posB.append(pos[1])
                    sizeA.append(size[0])
                    sizeB.append(size[1])
                    # print("p", posA)
                    
                    # print('resultR0' , resultR0)
                    # print('resultR1' , resultR1)
                    # print('resultR2' , resultR2)
                    # print('resultF0' , resultF0)
                    print('resultF1' , resultF1)
                    # print('resultF2' , resultF2)
                    # print('resultL0' , resultL0)
                    # print('resultL1' , resultL1)
                    # print('resultL2' , resultL2)
                    
                    
                    if resultL1 < 5 or resultL2 < 5 or resultL0 < 5 :
                        roll_disturbance = 1.0
                        
                    if resultR0 < 5 or resultR1 < 5 or resultR2 < 5  :
                        roll_disturbance = 1.0        
                    
                    if  resultF2 < 5 and   resultF1 < 5 :
                        
                            pitch_disturbance = 0.5 
                        
                    if  resultF0 < 5 and   resultF1 < 5 :
                        # if waypoints > abs(gps_v0):  
                            pitch_disturbance = 0.5 
                          
                    
                    if  resultF0 <= 5 and  210 < posA[1]  :
                        yaw_disturbance = 1.3
                    
                    if  resultF2 < 5 and   posA[1]  <150 :
                        yaw_disturbance = -1.3
                    
                    if  5 <= resultF1 < 5.5 and  150 < posA[1] < 210  :
                        pitch_disturbance = 2
                    
                    
                    
                    
                    # if resultL2 < 5 or resultL1 < 5 or resultL0 < 5 :  
                        # roll_disturbance = 1.0
                        
                    # if resultL0 <5 and resultF2 <5  :
                       # roll_disturbance = 1.0
                    
                    # if  resultF2 <5 and   resultF1 < 5 :
                        # pitch_disturbance = 0.5 
                        
                    # if  2.5 < resultF1 <5 :
                        # pitch_disturbance = 2
                        
                    # if  5 <= resultF1 < 5.5 and  150 < posA[1] < 210  :
                        # pitch_disturbance = 2
                                     
                    # if  resultF0 <5 and   resultF1 <= 5 :
                        # pitch_disturbance = 0.5       
                    # if resultR2 < 5 or resultR1 < 5 or resultR0 < 5 or  210 < posA[1]  :  
                        # yaw_disturbance = 0.7
                    
       
       
       
                    # if    resultL0 < 10 or resultL1 < 10  :
                               # roll_disturbance = 1.0
                            
                    # elif   resultF2 < 10 and resultF1 == inf :
                                # roll_disturbance = 1.0
                                
                    # elif  resultR0 < 10 or resultR1 < 10 :
                               # roll_disturbance = -1.0
                            
                    # elif   resultF0 < 10 and resultF1 == inf :
                                # roll_disturbance = -1.0 
                                
                    # elif  resultF1 < 10 and resultF == inf:
                                # pitch_disturbance = 1.3
                                
                    # elif  resultF1 <10 and  resultF2 < 10 :
                               # pitch_disturbance = 0.5
                               
                    # elif  resultF1 <10 and  resultF0 < 10 :
                               # pitch_disturbance = 0.5                               
                    
                    # elif resultF1 < 5 or resultF1 == inf  :
                            # pitch_disturbance = 1.3
                             
 
 
 
 
 
 
 
 
 
                    
                    # if sizeA[1] <= 200:
                        # if    resultL < 10 and posA[1] < 150  :
                               # roll_disturbance = 1.0
                            
                        # elif   resultF2 < 10:
                                # roll_disturbance = 1.0
                                
                        # elif  resultR < 10 and posA[1] > 210   :
                               # roll_disturbance = -1.0
                            
                        # elif   resultF0 < 10:
                                # roll_disturbance = -1.0 
                                
                        # elif  resultF1 < 10 and resultF == inf:
                                # pitch_disturbance = 1.3
                                
                        # elif 150 < posA[1] < 230 and resultF == inf :
                                # pitch_disturbance = 1.3
                             
                    # else :
                    
                        # if resultF < 5 or resultF == inf  :
                            # pitch_disturbance = 1.3
                            
                    
                    
                        # if  resultF2 < 4 or  resultL < 4:
                            # roll_disturbance = 1.0
                        # elif resultF0 < 4 or resultR < 4 : 
                            # roll_disturbance = -1.0
                        # pitch_disturbance = 0.5        
                        # elif resultF1 <2 :
                            # pitch_disturbance = 0.5     
                        
                        
                   
                        
                            
                        # elif resultR < 4 :
                            # roll_disturbance = -1.0
                        # elif resultF < 4 :
                             # pitch_disturbance = 2.0              
                    
                    
                    
                    
                    
                       # if minLidarR < 10 :
                           # roll_disturbance = 1.0
                       # elif minLidarF < 10 or minLidarML < 10 :
                           # pitch_disturbance = 2.0
                       # elif minLidarF < 10:
                           # pitch_disturbance = 2.0;
                       # elif minLidarL < 10:
                           # roll_disturbance = -1.0
                           
                    # else :
                        # pitch_disturbance = 1.0   
                        
                    
                    
                    
                    
                    # if sizeA[1] <= 200:
                        # if  posA[1] <= 120 :
                            # roll_disturbance = 1.0
                        
                        # elif posA[1] >= 180:
                            # roll_disturbance = -0.5
                        # elif 130 <= posA[1] <= 170 :
                            # pitch_disturbance = 2.0
                        # elif posA[1] >= 180 :
                            # roll_disturbance = -1.0
                    # else :
                            # pitch_disturbance = 1.0    
                            
                    # if 0 <= posA[1] <= 140 :
                        # roll_disturbance = 1.0
                    # elif 140 <= posA[1] <= 190 :
                        # pitch_disturbance = 2.0
                    # elif 190 <= posA[1] <= 360 :
                        # roll_disturbance = -1.0
                # else :
                       # pitch_disturbance = 2.0         
                # result = HD(lidar_vf , lidar_vr , lidar_vl ,posA )
                
                # if result[0] == False and result[1] == False:
                    # roll_disturbance = 1.0
                   
                # elif result[0] == True and result[1] == False:
                    # roll_disturbance = -1.0 
                                  
                # elif result[0] == False and result[1] == True :
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




