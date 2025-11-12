#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import tf2_ros
from tf2_ros import TransformException
from tf.transformations import euler_from_quaternion
import threading
import csv
import time 
import datetime
import serial
import rospkg
import numpy as np
from signal_detection.srv import GetSignalData, GetSignalDataResponse

target_frame = "map"
child_frame = "base_footprint"

class CachedPose:
    def __init__(self):
        self.lock = threading.Lock()
        self.x = rospy.get_param("/amcl/initial_pose_x")
        self.y = rospy.get_param("/amcl/initial_pose_y")
        self.yaw = rospy.get_param("/amcl/initial_pose_a")
        self.rssi_inst = -999
        self.avg_rssi = -999
        
    def update_pose(self, x, y, yaw):
        with self.lock:
            self.x = x
            self.y = y
            self.yaw = yaw
    
    def update_rssi(self, rssi_val, avg_rssi):
        with self.lock:
            self.rssi_val = rssi_val
            self.avg_rssi = avg_rssi

    def get(self):
        with self.lock:
            return self.x, self.y, self.yaw, self.rssi_val, self.avg_rssi
        
def quat_to_yaw(q):
    return euler_from_quaternion((q.x, q.y, q.z, q.w))[2]

def ewma(val, avg, alpha):
    return alpha*val + (1-alpha)*avg

def tf_updater_thread(cached_pose, stop_event, rate_hz=50.0):
    """
    Background thread: keeps cached_pose up-to-date by repeatedly
    looking up TARGET_FRAME -> CHILD_FRAME.
    Uses a short lookup timeout so the thread remains responsive.
    """
    tf_buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buf)

    rate = rospy.Rate(rate_hz)
    while not stop_event.is_set() and not rospy.is_shutdown():
        try:
            t = tf_buf.lookup_transform(target_frame, child_frame, rospy.Time(0), rospy.Duration(0.05))
            x = t.transform.translation.x
            y = t.transform.translation.y
            yaw = quat_to_yaw(t.transform.rotation)
            cached_pose.update(x, y, yaw)
        except Exception:
            # ignore lookup failures; we'll try again on the next loop
            pass
        rate.sleep()


def main():
    rospy.init_node("signal_service")

    start_time = time.time()
    prev_time = start_time  
    
    try:
        ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    except Exception as e:
        rospy.logerr("- signal service - Could not open serial port: %s", e)
        return
    
    tf_buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buf)
    
    cached_pose = CachedPose()
    stop_event = threading.Event()
    start_time = time.time()

    def handle_get_signal_data(req):
        x, y, yaw, rssi_val, avg_rssi = cached_pose.get()
        resp = GetSignalDataResponse()
        resp.x = x
        resp.y = y
        resp.yaw = yaw
        resp.rssi_val = rssi_val
        resp.avg_rssi = avg_rssi
        return resp

    # start TF updater thread
    tf_thread = threading.Thread(target=tf_updater_thread, args=(cached_pose, stop_event), daemon=True)
    tf_thread.start()
    rospy.loginfo("- signal service - Started TF updater thread")
    
    service = rospy.Service('get_signal_data', GetSignalData, handle_get_signal_data)
    rospy.loginfo("- signal service - Signal service 'get_signal_data' is ready")
    
    count = 0
    starting_avg = []
    
    try:
        while not rospy.is_shutdown():
            
            if ser.in_waiting:
                rssi_val = int(ser.readline().decode('utf-8').strip()) #.split(',')
                
                if count < 20:
                    starting_avg.append(rssi_val)
                    avg_rssi = rssi_val
                    count += 1
                elif count == 20: ## Calculate the starting average from the mean of the first 20 readings (whilst stood still)
                    avg_rssi = np.mean(starting_avg)
                    rospy.loginfo("- signal service - ewma start")
                    count +=1
                else:
                    avg_rssi = ewma(rssi_val, avg_rssi, 0.01)
                    
                
                cached_pose.update_rssi(rssi_val, int(avg_rssi))
                
                # current_time = time.time()
                # if current_time - prev_time >= 5:
                #     rospy.loginfo(cached_pose.get())
                #     prev_time = current_time
            else:
                time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        tf_thread.join(timeout=1.0)
        ser.close()
        rospy.loginfo("Shutting down node")
        

if __name__ == '__main__':
    main()