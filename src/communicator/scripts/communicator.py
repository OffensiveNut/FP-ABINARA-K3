#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray, Int32
from interpolation import interpolatesg, interpolatemg
import serial
import threading
from sensor_msgs.msg import JointState
import math 
import queue

PORT = '/dev/ttyACM0'
BAUDRATE = 9600

# Global publisher for dist topic
dist_pub = None

def send_serial(data_str):
    try:
        with serial.Serial(PORT, BAUDRATE, timeout=1) as ser:
            ser.write(data_str.encode())
    except serial.SerialException as e:
        rospy.logerr(f"Error sending data: {e}")

def receive_serial():
    global dist_pub
    try:
        with serial.Serial(PORT, BAUDRATE, timeout=1) as ser:
            rospy.loginfo(f"Listening on {PORT} at {BAUDRATE} baud...")
            while not rospy.is_shutdown():
                if ser.in_waiting > 0:
                    data = ser.readline().decode('utf-8').strip()
                    rospy.loginfo(f"Data received: {data}")
                    try:
                        dist_msg = Int32(int(data))
                        dist_pub.publish(dist_msg)
                    except ValueError as e:
                        rospy.logerr(f"Error converting data: {e}")
    except serial.SerialException as e:
        rospy.logerr(f"Error receiving data: {e}")

def send_dummy_dist():
    global dist_pub
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        try:
            dist_msg = Int32(data=300)
            dist_pub.publish(dist_msg)
            rate.sleep()
        except rospy.ROSInterruptException:
            break

def callback(data):
    # Extract joint positions from JointState message
    angles = data.position
    digital_units = [interpolatesg(angle) for angle in angles]
    rospy.loginfo(f'Sudut: {angles}')
    rospy.loginfo(f'Nilai: {digital_units}')
    formatted_units = ' '.join(f'{value:04d}' for value in digital_units)
    send_serial(formatted_units)

def main():
    global dist_pub
    
    # Initialize ROS node once
    rospy.init_node('communicator', anonymous=True)
    
    # Initialize publishers
    dist_pub = rospy.Publisher('dist', Int32, queue_size=10)
    
    # Create a subscriber to the "joint_states" topic, expecting messages of type JointState
    rospy.Subscriber('joint_states', JointState, callback)

    # Start a background thread to listen for incoming serial data
    receive_thread = threading.Thread(target=receive_serial, daemon=True)
    receive_thread.start()
    
    send_dist_thread = threading.Thread(target=send_serial, daemon=True)
    send_dist_thread.start()

    # Spin to keep the node running and processing messages
    rospy.spin()

if __name__ == '__main__':
    # Start the main function
    main()
