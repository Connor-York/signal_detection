#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from std_msgs.msg import Float32
import csv
import time 
import datetime
import serial
import rospkg

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

def pose_callback(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    global current_pose 
    current_pose = [x,y]


if __name__ == '__main__':
    
    rospy.init_node("signal_mapping")
    pose_subscriber = rospy.Subscriber("amcl_pose",PoseWithCovarianceStamped,pose_callback)
    
    with open(path, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['timestamp','pose_x','pose_y','rssi'])
        
        while not rospy.is_shutdown():

            
            if ser.in_waiting:
                arduino_output = ser.readline().decode('utf-8').strip()
                timestamp = round(time.time() - start_time,2)
                
                datapack = [timestamp, current_pose[0], current_pose[1], arduino_output]
                writer.writerow(datapack)
        

