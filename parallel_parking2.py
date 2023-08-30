#!/usr/bin/env

import time
import random
import math 
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Int64, Int32, Float64

started = False
desired_Position = 0.0
lidar_distance = 0.0
length_spot = 0.0
depth_spot = 0.0
speed = 0.0
imu=0.0
ultra_distanceB = 0.0
ultra_distanceF = 0.0
car_width = 0.70 #cm
car_length = 1.15 #cm

pub = rospy.Publisher('/PWM_Values_S',Int32,queue_size = 10)    #speed
pub1 = rospy.Publisher('/PWM_Values_A',Int32,queue_size = 10)   #angle

rospy.init_node('park_node', anonymous = True)
 
rate = rospy.Rate(10000)

def nodes():

      
      
      #rospy.init_node('lidar', anonymous = True)
      rospy.Subscriber('Real_Speed',Float32,callback)
      #rospy.Subscriber('Steering_angle_topic',Float32,callback1)
      #rospy.Subscriber('lidar_value_topic',Float32,callback2)
      rospy.Subscriber('/scan', LaserScan, callback2)
      #rospy.Subscriber("angle", Float64, callback_imu)
      rospy.Subscriber('br',Float64, callback_ultraB)
      rospy.Subscriber('fr',Float64, callback_ultraF)
      parking(True)
      #rospy.Subscriber('RealSpeedL',Float32,callback)
      timer = rospy.Timer(rospy.Duration(0.01),parking)
      #timer1 = rospy.Timer(rospy.Duration(0.01),parallel_park)
      rospy.spin()
      timer.shutdown()
      #timer1.shutdown()
#def callback_imu(data):
 #   global imu
  #  rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
  #  imu=data.data

def callback(data):
    global started,speed

    
    speed = data.data
    #print ("RealValue Received",speed)
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", speed)
    

def callback1(data):
    global desired_Position
    desired_Position = data.data
 

def callback2(data):
    
    global lidar_distance, started
    
    lidar_distance = data.ranges[0]
    
    if (not started):
        started = True

def callback_ultraB(distance):
    global ultra_distanceB
    
    ultra_distanceB = distance.data


    
   # print ("UltraB Received",ultra_distanceB)
  #  rospy.loginfo(rospy.get_caller_id() + "I heard %s", ultra_distanceB)
    
    
def callback_ultraF(distance):
    global ultra_distanceF
    
    ultra_distanceF = distance.data
    
    #print ("UltraF Received",ultra_distanceF)
   # rospy.loginfo(rospy.get_caller_id() + "I heard %s", ultra_distanceF)
    
    

def parking(event=True):
    global started,pub,speed,lidar_distance,desired_Position,car_length,car_width,pub1, length_spot, depth_spot, ultra_distanceB, ultra_distanceF
    
    if(started):
             
             pub1.publish(0)    #PWM Steering
             pub.publish(100)    #PWM speed
             
             time.sleep(2)
             #rate.sleep()
             if (lidar_distance > 1.2 * car_width):
                  
                    startTime = time.time()             #timer
                    depth_spot = lidar_distance
                    while(lidar_distance > 1.2 * car_width):
                       print(f"lidar : {lidar_distance}")
                       print(f"Speed : {speed}")

                    endTime = time.time()
                    elapsedTime = endTime - startTime
                    length_spot = elapsedTime * speed
                    print (f"car speed: {speed} ")
                    print (f"length_spot: {length_spot} , depth spot: {depth_spot} ")
             # length_spot > (1.2 * car_length) and 1.2 * car_width
             if ( depth_spot > (1.2 * car_width)):
                    
                    print("Parallel Parking")
                    
                    pub.publish(80)  #stop car
                    pub1.publish(0)
                    rospy.sleep(2)
                    pub1.publish(30)
                    rospy.sleep(2)
                    #rate.sleep()
      
                    while(1):
                         
                         print("park")
                         pub.publish(0)   #speed backwards
                         rospy.sleep(2)
                         #rate.sleep()
                         pub1.publish(-30)   #max angle right
                         #rate.sleep()
                         
                         #if (ultra_distanceB <= 20):
                         rospy.sleep(5)
               
                         break
                         
                    while(1):
                         
                         print("park2")
                         pub.publish(-70)   #speed backwards
                         #rate.sleep()
                         #pub1.publish(-30)   #max angle right
                         #rate.sleep()
                         
                         #if (ultra_distanceB <= 20):
                         rospy.sleep(7.5)
               
                         break
                              
                    while(1):
                         
                         pub.publish(0)
                         
                         pub1.publish(30)
                    
                         time.sleep(7)
                         
                         break
                         
                    while(1):
                         
                         

                         
                         #time.sleep(10)
                         
                         
                         pub.publish(-80)   #speed backwards
                         pub1.publish(30)
                         rospy.sleep(8)
                         print("s2")
                         # pub1.publish(30)   #max angle left
                         #rate.sleep()
                         rospy.Subscriber('br',Float64, callback_ultraB)
                         rospy.Subscriber('fr',Float64, callback_ultraF)

                         print ("UltraB Received",ultra_distanceB)
                         if (ultra_distanceB <= 20):
                          print("ssssssssssssssssssssssssssssssssss")
                         
                         #if (imu >= 179.9 ):
                         break

                    '''while(1):
                         
                         print("s3")
                         
                         pub.publish(0)
                         pub.publish(0)
                         pub.publish(0)
                         pub.publish(0)
                         time.sleep(5)
                         print("t1")
                         pub1.publish(-30)
                         time.sleep(2)
                         print("t2")
                         break'''

                    while(1):
                         
                         print("S3")
                         pub.publish(0)
                         
                         pub1.publish(0)
                         """for i in range(10000):
                             print("s")
                             pub.publish(0)
                             pub1.publish(-30)"""
                 
                    
                         #time.sleep(10)
                         
                         #break
                                   
                         
                    while(1):
                         print("s4")
                         pub.publish(80)   #speed Upwards
                         pub1.publish(-30)   #max angle right
                         #rate.sleep()
                         rospy.Subscriber('br',Float64, callback_ultraB)
                         rospy.Subscriber('fr',Float64, callback_ultraF)
                         if (ultra_distanceF <= 20):
                         #if (imu >= 179.9 ):
                          break
                    
                         
                    while(1):
                         
                         pub.publish(0)   #speed zero
                         pub1.publish(0)   #zero

                         break    
 
           


if __name__ == '__main__':

   try:
       print ("Running")
       nodes()
   except rospy.ROSInterruptException:
        pass
