#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from std_msgs.msg import String
import csv
import time 
import datetime
import serial
import rospkg
import json

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

rp = rospkg.RosPack()
package_path = rp.get_path('signal_detection')

csv_name = ('Signal_Map_Test_' + datetime.datetime.now().strftime("%H:%M:%S") + ".csv")
path = (package_path + '/logs/' + csv_name)

start_time = time.time()
prev_time = start_time

initial_x = rospy.get_param("/amcl/initial_pose_x")
initial_y = rospy.get_param("/amcl/initial_pose_y")

current_pose = [initial_x, initial_y]
patrol_status = [0,0] # start goal (0 assumed), patrol count (0 known)

def pose_callback(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    global current_pose 
    current_pose = [x,y]
    
def status_callback(msg):
    global patrol_status
    patrol_status = msg.data.strip().split(',')
    


if __name__ == '__main__':
    
    rospy.init_node("signal_mapping")
    pose_subscriber = rospy.Subscriber("/amcl_pose",PoseWithCovarianceStamped,pose_callback)
    status_subscriber = rospy.Subscriber("/patrol_status",String,status_callback)

    with open(path, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['timestamp','curr_patrol','curr_goal','pose_x','pose_y','xbee','lora'])
        
        while not rospy.is_shutdown():

            
            if ser.in_waiting:
                arduino_output = ser.readline().decode('utf-8').strip().split(',')
                timestamp = round(time.time() - start_time,2)
                
                
                datapack = [timestamp, patrol_status[1], patrol_status[0], current_pose[0], current_pose[1], int(arduino_output[0]), int(arduino_output[1])]
                writer.writerow(datapack)
                
                current_time = time.time()
                if current_time - prev_time >= 1:
                    rospy.loginfo(datapack)
                    prev_time = current_time
        

