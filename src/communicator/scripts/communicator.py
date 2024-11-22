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

# Initialize a queue for serial data
serial_queue = queue.Queue()

def send_serial():
    try:
        with serial.Serial(PORT, BAUDRATE, timeout=1) as ser:
            rospy.loginfo(f"Connected to serial port: {PORT} at {BAUDRATE} baud.")
            while not rospy.is_shutdown():
                try:
                    # Get data from the queue with a timeout
                    data_str = serial_queue.get(timeout=1)
                    ser.write(data_str.encode())
                    rospy.loginfo(f"Sent over serial: {data_str}")
                except queue.Empty:
                    continue
                except serial.SerialException as e:
                    rospy.logerr(f"Error sending data: {e}")
                except Exception as e:
                    rospy.logerr(f"Unexpected error in send_serial: {e}")
    except serial.SerialException as e:
        rospy.logerr(f"Error opening serial port: {e}")

def receive_serial():
    global dist_pub
    try:
        with serial.Serial(PORT, BAUDRATE, timeout=1) as ser:
            rospy.loginfo(f"Listening on {PORT} at {BAUDRATE} baud...")
            while not rospy.is_shutdown():
                try:
                    if ser.in_waiting > 0:
                        data = ser.readline().decode('utf-8').strip()
                        rospy.loginfo(f"Data received: {data}")
                        try:
                            # Directly convert the received data to an integer
                            dist_value = int(data)
                            dist_msg = Int32(data=dist_value)
                            dist_pub.publish(dist_msg)
                        except ValueError as e:
                            rospy.logerr(f"Error converting data: {e}")
                except serial.SerialException as e:
                    rospy.logerr(f"Serial read error: {e}")
                except Exception as e:
                    rospy.logerr(f"Unexpected error in receive_serial: {e}")
    except serial.SerialException as e:
        rospy.logerr(f"Error opening serial port: {e}")

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
    
    # Enqueue the formatted string for sending over serial
    serial_queue.put(formatted_units)

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
    
    # Start a background thread to send serial data
    send_thread = threading.Thread(target=send_serial, daemon=True)
    send_thread.start()

    # Spin to keep the node running and processing messages
    rospy.spin()

    # Ensure threads are properly joined before exiting
    receive_thread.join()
    send_thread.join()

if __name__ == '__main__':
    # Start the main function
    main()
