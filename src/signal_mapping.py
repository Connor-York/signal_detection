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

rp = rospkg.RosPack()
package_path = rp.get_path('signal_detection')

csv_name = ('Signal_Map_Test_' + datetime.datetime.now().strftime("%H:%M:%S") + ".csv")
path = (package_path + '/logs/' + csv_name)

patrol_status = [0,0] # start goal (0 assumed), patrol count (0 known)

target_frame = "map"
child_frame = "base_footprint"

class CachedPose:
    def __init__(self):
        self.lock = threading.Lock()
        self.x = rospy.get_param("/amcl/initial_pose_x")
        self.y = rospy.get_param("/amcl/initial_pose_y")
        self.yaw = rospy.get_param("/amcl/initial_pose_a")
        
    def update(self, x, y, yaw):
        with self.lock:
            self.x = x
            self.y = y
            self.yaw = yaw

    def get(self):
        with self.lock:
            return self.x, self.y, self.yaw
    
def status_callback(msg):
    global patrol_status
    patrol_status = msg.data.strip().split(',')
    
def quat_to_yaw(q):
    return euler_from_quaternion((q.x, q.y, q.z, q.w))[2]


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
    rospy.init_node("signal_mapping")
    status_subscriber = rospy.Subscriber("/patrol_status",String,status_callback)
    
    start_time = time.time()
    prev_time = start_time  
    
    try:
        ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    except Exception as e:
        rospy.logerr("Could not open serial port: %s", e)
        return
    
    tf_buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buf)
    
    cached_pose = CachedPose()
    stop_event = threading.Event()
    start_time = time.time()

    # start TF updater thread
    tf_thread = threading.Thread(target=tf_updater_thread, args=(cached_pose, stop_event), daemon=True)
    tf_thread.start()
    rospy.loginfo("Started TF updater thread")

    with open(path, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['timestamp','curr_patrol','curr_goal','pose_x','pose_y','pose_a','xbee']) #'m5','xbee','lora'])
        
        try:
            while not rospy.is_shutdown():
                
                if ser.in_waiting:
                    arduino_output = ser.readline().decode('utf-8').strip() #.split(',')
                    timestamp = round(time.time() - start_time,2)
                    x, y, yaw = cached_pose.get()
                    datapack = [timestamp, patrol_status[1], patrol_status[0], x, y, yaw, int(arduino_output)] #int(arduino_output[0]), int(arduino_output[1])]
                    rospy.loginfo(datapack)
                    writer.writerow(datapack)
                    
                    current_time = time.time()
                    if current_time - prev_time >= 1:
                        rospy.loginfo(datapack)
                        prev_time = current_time
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
        

